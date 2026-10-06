#!/usr/bin/env python3
"""A chase-camera view of the FAST-LIO map, rendered on the GPU and streamed as video.

The second stream of the MAVLink camera (QGC lists it next to the D555 RGB): the 5 cm
voxel map from lio_map coloured by height, the live Avia scan (white) and the registered
E1R (magenta), both sensors' fields of view, the flight trail and the X950 itself (its
URDF meshes), seen from behind and above the aircraft. Rendered headless (EGL, OpenGL 4.3
core on the Orin's GPU) with eye-dome lighting, converted and flipped by nvvidconv,
encoded by NVENC (H.265) and served over RTSP.

The camera rides a boom behind the aircraft that lengthens and steepens with height above
the ground, so that the ground straight below (the E1R's footprint) stays in the lower
part of the frame and the Avia's forward view above it, at any altitude. Map points
between the camera and the aircraft are cut away, and so is everything more than a metre
above the aircraft while it is enclosed (indoors, under a canopy): a dollhouse view.

Nothing heavy runs while nobody watches: only FAST-LIO's odometry is followed, for the
trail. The first RTSP client starts the rendering and the map, scan and E1R subscriptions
(lio_map then sends the whole fine map); the last one stops them.
"""
import argparse
import collections
import ctypes
import json
import math
import os
import signal
import struct
import threading
import time
import xml.etree.ElementTree as ET

import numpy as np

OFFSET = 1 << 20          # voxel key packing, as in lio_map: 21 bits per axis
AVIA_FOV = (70.4, 77.2)   # deg, horizontal x vertical (its rosette fills an ellipse)
E1R_FOV = (120.0, 90.0)   # deg, horizontal x vertical


# ----- geometry -----------------------------------------------------------------------

def quat_matrix(x, y, z, w):
    """3x3 rotation matrix of a unit quaternion."""
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def rpy_matrix(roll, pitch, yaw):
    """URDF fixed-axis roll, pitch, yaw: R = Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def transform(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0)):
    t = np.eye(4)
    t[:3, :3] = rpy_matrix(*rpy)
    t[:3, 3] = xyz
    return t


def matrix_quat(r):
    """Unit quaternion (x, y, z, w) of a 3x3 rotation matrix."""
    t = np.trace(r)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        q = ((r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s, 0.25 * s)
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2
        q = (0.25 * s, (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s, (r[2, 1] - r[1, 2]) / s)
    elif r[1, 1] > r[2, 2]:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2
        q = ((r[0, 1] + r[1, 0]) / s, 0.25 * s, (r[1, 2] + r[2, 1]) / s, (r[0, 2] - r[2, 0]) / s)
    else:
        s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2
        q = ((r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s, 0.25 * s, (r[1, 0] - r[0, 1]) / s)
    q = np.asarray(q, float)
    return tuple(float(v) for v in q / np.linalg.norm(q))


def yaw_of(q):
    x, y, z, w = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def look_at(eye, target, up=(0.0, 0.0, 1.0)):
    """OpenGL view matrix: the camera at eye looking at target, down its -z axis."""
    eye, target, up = (np.asarray(v, float) for v in (eye, target, up))
    f = target - eye
    f /= np.linalg.norm(f)
    s = np.cross(f, up)
    s /= np.linalg.norm(s)
    u = np.cross(s, f)
    m = np.eye(4)
    m[0, :3], m[1, :3], m[2, :3] = s, u, -f
    m[:3, 3] = -m[:3, :3] @ eye
    return m


def perspective(fovy_deg, aspect, near, far):
    f = 1.0 / math.tan(math.radians(fovy_deg) / 2)
    m = np.zeros((4, 4))
    m[0, 0], m[1, 1] = f / aspect, f
    m[2, 2], m[2, 3] = (far + near) / (near - far), 2 * far * near / (near - far)
    m[3, 2] = -1.0
    return m


def slerp(q0, q1, u):
    q0, q1 = np.asarray(q0, float), np.asarray(q1, float)
    d = float(np.dot(q0, q1))
    if d < 0:
        q1, d = -q1, -d
    if d > 0.9995:
        q = q0 + u * (q1 - q0)
    else:
        th = math.acos(d)
        q = (math.sin((1 - u) * th) * q0 + math.sin(u * th) * q1) / math.sin(th)
    return q / np.linalg.norm(q)


def base_from_imu(position, q, mount):
    """base_link's pose for FAST-LIO's IMU pose, mount being the IMU's 4x4 pose in base_link
    (the URDF's avia_imu, as the lio bridge has it): T_w_base = T_w_imu mount^-1.
    Returns (position (3,), quaternion (x, y, z, w))."""
    t = np.eye(4)
    t[:3, :3], t[:3, 3] = quat_matrix(*q), position
    base = t @ np.linalg.inv(mount)
    return base[:3, 3], matrix_quat(base[:3, :3])


class PoseBuffer:
    """Recent base_link poses (stamp s, position, quaternion), interpolated for a smooth view.

    Stamps are in the Avia's clock, so what to show when is derived from arrival times
    (monotonic): the smallest arrival - stamp seen lately is the pipeline's latency, and
    showing one pose interval (plus jitter) later than that means the pose after the
    shown instant has almost always arrived, so the view interpolates instead of
    stepping at FAST-LIO's 10 Hz."""

    def __init__(self, keep=64):
        self.poses = collections.deque(maxlen=keep)
        self.latency = collections.deque(maxlen=50)
        self.lock = threading.Lock()

    def add(self, t, position, q, arrival=None):
        """False if the pose was dropped (not newer than the last one)."""
        with self.lock:
            if self.poses and t <= self.poses[-1][0]:
                if t < self.poses[-1][0] - 1.0:
                    self.poses.clear()            # time went back: FAST-LIO restarted, or a replay
                    self.latency.clear()
                else:
                    return False
            self.poses.append((t, np.asarray(position, float), np.asarray(q, float)))
            if arrival is not None:
                self.latency.append(arrival - t)
            return True

    def newest(self):
        with self.lock:
            return self.poses[-1] if self.poses else None

    def clear(self):
        with self.lock:
            self.poses.clear()
            self.latency.clear()

    def render_time(self, now):
        """The stamp to show at monotonic time now (None before the first pose)."""
        with self.lock:
            if not self.poses:
                return None
            if len(self.poses) < 3 or not self.latency:
                return self.poses[-1][0]
            stamps = np.array([p[0] for p in list(self.poses)[-20:]])
            interval = float(np.median(np.diff(stamps)))
            return now - min(self.latency) - 1.3 * interval - 0.02

    def at(self, t):
        """(position, quaternion) at stamp t: interpolated inside the buffer, clamped outside."""
        with self.lock:
            poses = list(self.poses)
        if not poses or t is None:
            return None
        if t >= poses[-1][0]:
            return poses[-1][1], poses[-1][2]
        if t <= poses[0][0]:
            return poses[0][1], poses[0][2]
        for (t0, p0, q0), (t1, p1, q1) in zip(poses, poses[1:]):
            if t0 <= t <= t1:
                u = (t - t0) / (t1 - t0)
                return p0 + u * (p1 - p0), slerp(q0, q1, u)
        return poses[-1][1], poses[-1][2]


def smoothstep(x):
    x = min(max(x, 0.0), 1.0)
    return x * x * (3 - 2 * x)


class ChaseCamera:
    """A boom behind the aircraft along its (smoothed) heading, the horizon level.

    Framing, with v the vertical screen position (-1 top, +1 bottom): the aircraft at
    aircraft_v, just above the centre; the ground straight below it, the middle of the
    E1R's footprint, no lower than nadir_v; the Avia's forward view fills the top. The
    boom elevation phi steepens from elevation[0] to elevation[1] as the height above
    the ground grows to steepen_agl (more of the footprint, less sky), the pitch follows
    from the aircraft's place in the frame, and the length from the nadir's: a boom of
    length L at elevation phi sees the ground at depth agl below the aircraft at
    atan((agl + L sin phi) / (L cos phi)) below the horizon, inside the frame while
    L >= agl / (tan(pitch + atan(nadir_v tan(fovy/2))) cos phi - sin phi).
    First-order lags on the heading, the position and the height above ground swing the
    view through turns and climbs instead of snapping."""

    def __init__(self, fovy=50.0, distance=7.0, max_distance=90.0, elevation=(25.0, 45.0),
                 steepen_agl=30.0, aircraft_v=-0.1, nadir_v=0.8, yaw_tau=0.8, pos_tau=0.25, agl_tau=1.0):
        self.tan_half = math.tan(math.radians(fovy) / 2)
        self.distance, self.max_distance = distance, max_distance
        self.elevation, self.steepen_agl = elevation, steepen_agl
        self.aircraft_v, self.nadir_v = aircraft_v, nadir_v
        self.yaw_tau, self.pos_tau, self.agl_tau = yaw_tau, pos_tau, agl_tau
        self.yaw = self.pos = self.agl = None
        self.length = distance

    def boom(self, agl):
        """(elevation, length, pitch) for a height agl above the ground; angles in radians."""
        e0, e1 = self.elevation
        phi = math.radians(e0 + (e1 - e0) * smoothstep(agl / self.steepen_agl))
        pitch = phi - math.atan(self.aircraft_v * self.tan_half)
        k = math.tan(pitch + math.atan(self.nadir_v * self.tan_half)) * math.cos(phi) - math.sin(phi)
        length = agl / k if k > 1e-3 else self.max_distance
        return phi, min(max(self.distance, length), self.max_distance), pitch

    def update(self, position, yaw, agl, dt):
        """(eye, target) for the aircraft at position with heading yaw, agl metres above the
        ground, dt seconds after the last call."""
        position, agl = np.asarray(position, float), max(float(agl), 0.0)
        if self.yaw is None or dt <= 0 or dt > 5.0:
            self.yaw, self.pos, self.agl = yaw, position.copy(), agl
        else:
            err = math.atan2(math.sin(yaw - self.yaw), math.cos(yaw - self.yaw))
            self.yaw += (1 - math.exp(-dt / self.yaw_tau)) * err
            self.pos += (1 - math.exp(-dt / self.pos_tau)) * (position - self.pos)
            self.agl += (1 - math.exp(-dt / self.agl_tau)) * (agl - self.agl)
        phi, self.length, pitch = self.boom(self.agl)
        forward = np.array([math.cos(self.yaw), math.sin(self.yaw), 0.0])
        up = np.array([0.0, 0.0, 1.0])
        eye = self.pos - forward * self.length * math.cos(phi) + up * self.length * math.sin(phi)
        return eye, eye + (forward * math.cos(pitch) - up * math.sin(pitch)) * self.length


class Trail:
    """The flight path, one point every `step` metres, capped (oldest half thinned when full)."""

    def __init__(self, step=0.1, cap=100_000):
        self.step, self.cap = step, cap
        self.points = np.zeros((cap, 3), np.float32)
        self.count = 0
        self.version = 0

    def add(self, p):
        p = np.asarray(p, np.float32)
        if self.count and np.linalg.norm(p - self.points[self.count - 1]) < self.step:
            return False
        if self.count == self.cap:
            half = self.cap // 2
            self.points[:half // 2] = self.points[:half:2]        # every other point of the older half
            self.points[half // 2:half // 2 + half] = self.points[half:]
            self.count = half // 2 + half
        self.points[self.count] = p
        self.count += 1
        self.version += 1
        return True

    def clear(self):
        self.count = 0
        self.version += 1

    def view(self):
        return self.points[:self.count]


def voxel_keys(xyz, voxel):
    """One int64 per point: its voxel's (ix, iy, iz), packed 21 bits each (as lio_map)."""
    idx = np.floor(xyz / voxel).astype(np.int64) + OFFSET
    if idx.size and (idx.min() < 0 or idx.max() >= 1 << 21):
        raise ValueError("point outside the map's index range")
    return (idx[:, 0] << 42) | (idx[:, 1] << 21) | idx[:, 2]


class VoxelDedup:
    """One point per voxel, as lio_map keeps them, so that its whole-map resends (to every
    subscriber, whenever a new one appears) add nothing twice. The keys live in a sorted
    int64 array: 8 bytes per voxel where a Python set takes ~70 (4 M voxels: 32 MB)."""

    def __init__(self, voxel=0.05, cap=4_000_000):
        self.voxel, self.cap = float(voxel), int(cap)
        self.keys = np.zeros(0, np.int64)

    def __len__(self):
        return len(self.keys)

    def add(self, points):
        """points (n, 4) float32 -> those in voxels not seen before (the first point of each)."""
        if not len(points) or len(self.keys) >= self.cap:
            return points[:0]
        points = points[np.isfinite(points[:, :3]).all(axis=1)]
        if not len(points):
            return points
        keys, first = np.unique(voxel_keys(points[:, :3].astype(np.float64), self.voxel), return_index=True)
        if len(self.keys):
            at = np.searchsorted(self.keys, keys)
            seen = self.keys[np.minimum(at, len(self.keys) - 1)] == keys
            keys, first, at = keys[~seen], first[~seen], at[~seen]
        else:
            at = np.zeros(len(keys), np.int64)
        room = self.cap - len(self.keys)
        keys, first, at = keys[:room], first[:room], at[:room]
        self.keys = np.insert(self.keys, at, keys)
        return points[first]

    def clear(self):
        self.keys = np.zeros(0, np.int64)


class Promoter:
    """Shows a map point only once its cell (0.5 m across, 0.25 m tall) holds `need` of them,
    then all of the cell's points at once. A surface fills its cells within a scan or two.
    The Avia's stray returns never do: about 0.3 % of its points indoors, at random ranges
    along real beams out to 430 m, and not flagged in the Livox tag's noise bits. Each lands
    in a voxel of its own, deduplicates against nothing, and would otherwise settle as dust
    over the whole view. Held cells are capped, the oldest dropped first."""

    def __init__(self, need=3, cell=0.5, level=0.25, cap=400_000):
        self.need, self.cell, self.level, self.cap = int(need), float(cell), float(level), int(cap)
        self.clear()

    def clear(self):
        self.held = {}                  # cell -> [points so far, [arrays]]
        self.shown = set()              # cells promoted

    def _keys(self, xyz):
        ix, iy = (np.floor(xyz[:, k] / self.cell).astype(np.int64) + OFFSET for k in (0, 1))
        iz = np.floor(xyz[:, 2] / self.level).astype(np.int64) + OFFSET
        return (ix << 42) | (iy << 21) | iz

    def add(self, points):
        """points (n, 4) -> those to show now: in promoted cells, or completing one."""
        if self.need <= 1 or not len(points):
            return points
        keys = self._keys(points[:, :3].astype(np.float64))
        order = np.argsort(keys, kind="stable")
        sorted_keys = keys[order]
        starts = np.flatnonzero(np.r_[True, sorted_keys[1:] != sorted_keys[:-1]])
        ends = np.r_[starts[1:], len(sorted_keys)]
        out = []
        for s, e, k in zip(starts.tolist(), ends.tolist(), sorted_keys[starts].tolist()):
            group = points[order[s:e]]
            if k in self.shown:
                out.append(group)
                continue
            entry = self.held.setdefault(k, [0, []])
            entry[0] += e - s
            entry[1].append(group)
            if entry[0] >= self.need:
                del self.held[k]
                self.shown.add(k)
                out.extend(entry[1])
        if len(self.held) > self.cap:
            for k in list(self.held)[:len(self.held) - self.cap + self.cap // 10]:
                del self.held[k]
        return np.concatenate(out) if out else points[:0]


class Occupancy:
    """The map's occupied cells, 0.5 m columns of 0.25 m levels, as one bit per level per
    column: enough to tell whether the aircraft is enclosed (a ceiling, a canopy) without a
    3D index."""

    LEVELS_BELOW = 1024                 # level 1024 is z in [0, level): +-256 m at 0.25 m

    def __init__(self, cell=0.5, level=0.25):
        self.cell, self.level = float(cell), float(level)
        self.columns = {}
        self.lock = threading.Lock()

    @staticmethod
    def _column(ix, iy):
        return ((ix + OFFSET) << 21) | (iy + OFFSET)

    def _level(self, z):
        return min(max(math.floor(z / self.level) + self.LEVELS_BELOW, 0), 2 * self.LEVELS_BELOW - 1)

    def add(self, xyz):
        if not len(xyz):
            return
        xyz = np.asarray(xyz, np.float64)
        ix, iy = (np.floor(xyz[:, k] / self.cell).astype(np.int64) + OFFSET for k in (0, 1))
        level = np.clip(np.floor(xyz[:, 2] / self.level).astype(np.int64) + self.LEVELS_BELOW,
                        0, 2 * self.LEVELS_BELOW - 1)
        keys = np.unique((((ix << 21) | iy) << 11) | level)
        columns, levels = (keys >> 11).tolist(), (keys & 2047).tolist()
        with self.lock:
            get = self.columns.get
            for c, b in zip(columns, levels):
                self.columns[c] = get(c, 0) | (1 << b)

    def clear(self):
        with self.lock:
            self.columns = {}

    def enclosure(self, x, y, z, radius, clear=(0.1, 0.6), roof=8.0):
        """Whether something hangs over the aircraft at (x, y, z): (columns within radius that
        hold map data, those of them with a ceiling: empty from clear[0] to clear[1] above z
        and occupied above that, up to roof). A wall, a trunk or a facade fills the clear
        band, so standing next to one is not being indoors. Nor is the floor: a 0.25 m
        level reaches at most a quarter metre above it."""
        c = self.cell
        ix, iy, n = math.floor(x / c), math.floor(y / c), int(math.ceil(radius / c))
        g0, g1, top = self._level(z + clear[0]), self._level(z + clear[1]), self._level(z + roof)
        gap = ((1 << (g1 - g0 + 1)) - 1) << g0
        over = ((1 << max(top - g1, 0)) - 1) << (g1 + 1)
        known = covered = 0
        with self.lock:
            for dx in range(-n, n + 1):
                for dy in range(-n, n + 1):
                    if dx * dx + dy * dy > n * n:
                        continue
                    m = self.columns.get(self._column(ix + dx, iy + dy), 0)
                    if m:
                        known += 1
                        covered += not m & gap and bool(m & over)
        return known, covered


def scan_enclosed(scan, position, max_p90=12.0, above=1.0, share=0.15, minimum=1000):
    """Whether the live Avia scan looks like the inside of a room: nine in ten returns closer
    than max_p90 (outdoors the far ground and the trees push that out to tens of metres)
    and at least `share` of them more than `above` over the aircraft (a ceiling, upper
    walls). It needs no map, so it works from the first scan on the bench, where the Avia
    (38.6 deg up at most) has not seen the ceiling over the aircraft."""
    if scan is None or len(scan) < minimum:
        return False
    d = scan[:, :3] - np.asarray(position, np.float32)
    r = np.linalg.norm(d, axis=1)
    ok = np.isfinite(r)
    if ok.sum() < minimum:
        return False
    return float(np.percentile(r[ok], 90)) < max_p90 and float(np.mean(d[ok, 2] > above)) >= share


def nadir_ground(points, x, y, z, radius, minimum=20, max_radius=12.0):
    """Median height of the points within radius (horizontally) of (x, y) and at least 0.1 m
    below z: the ground under the aircraft, from the E1R. The radius doubles until there are
    enough of them (an E1R frame's points thin out with height); None if never."""
    if points is None or not len(points):
        return None
    below = points[(points[:, 2] < z - 0.1) & np.isfinite(points[:, 2])]
    d2 = (below[:, 0] - x) ** 2 + (below[:, 1] - y) ** 2
    while True:
        near = below[d2 < radius * radius, 2]
        if len(near) >= minimum:
            return float(np.median(near))
        if radius >= max_radius:
            return None
        radius = min(2 * radius, max_radius)


def local_heights(sample, centre, radius=80.0, minimum=500):
    """Heights of the sampled map points within radius (horizontally) of centre, or all of
    them if too few: the colour range follows the ground the aircraft is over."""
    d2 = (sample[:, 0] - centre[0]) ** 2 + (sample[:, 1] - centre[1]) ** 2
    near = sample[d2 < radius * radius, 2]
    return near if len(near) >= minimum else sample[:, 2]


def robust_range(z, lo=2.0, hi=98.0, minimum=2.0):
    """Height colour range from percentiles, at least `minimum` metres wide."""
    if not len(z):
        return -1.0, 5.0
    a, b = (float(v) for v in np.percentile(z, [lo, hi]))
    if b - a < minimum:
        mid = (a + b) / 2
        a, b = mid - minimum / 2, mid + minimum / 2
    return a, b


# ----- overlay geometry (line lists, unit length along the sensor's +x) -----------------

def cone_lines(fov, segments=36):
    """An elliptical cone: its far ellipse and four edge rays."""
    a, b = (math.tan(math.radians(v) / 2) for v in fov)
    t = np.linspace(0, 2 * math.pi, segments + 1)
    ring = np.stack([np.ones_like(t), a * np.cos(t), b * np.sin(t)], axis=1)
    edges = np.stack([ring[:-1], ring[1:]], axis=1).reshape(-1, 3)
    rays = np.array([v for k in range(4) for v in ((0, 0, 0), ring[k * segments // 4])], float)
    return np.concatenate([edges, rays])


def pyramid_lines(fov):
    """A rectangular frustum: four edge rays and the far rectangle."""
    a, b = (math.tan(math.radians(v) / 2) for v in fov)
    c = [(1, a, b), (1, -a, b), (1, -a, -b), (1, a, -b)]
    return np.array([v for i in range(4) for v in ((0, 0, 0), c[i], c[i], c[(i + 1) % 4])], float)


def ring_lines(radius, segments=32):
    t = np.linspace(0, 2 * math.pi, segments + 1)
    ring = np.stack([radius * np.cos(t), radius * np.sin(t), np.zeros_like(t)], axis=1)
    return np.stack([ring[:-1], ring[1:]], axis=1).reshape(-1, 3)


def place(lines, pose, mount, length):
    """Unit sensor lines scaled to length and moved into the world through the aircraft's pose and the mount."""
    t = pose @ mount
    return (lines * length) @ t[:3, :3].T + t[:3, 3]


# ----- the URDF -----------------------------------------------------------------------

def read_stl(path):
    """Triangles (n, 3, 3) float32 of a binary or ASCII STL."""
    with open(path, "rb") as f:
        data = f.read()
    if len(data) >= 84:
        n = struct.unpack_from("<I", data, 80)[0]
        if 84 + 50 * n == len(data):
            rec = np.frombuffer(data, dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]),
                                count=n, offset=84)
            return rec["v"].astype(np.float32)
    verts = [line.split()[1:4] for line in data.decode(errors="replace").splitlines()
             if line.strip().startswith("vertex")]
    return np.array(verts, np.float32).reshape(-1, 3, 3)


def _floats(text, default):
    return [float(v) for v in text.split()] if text else list(default)


def _origin(element):
    o = element.find("origin")
    return transform(_floats(o.get("xyz") if o is not None else None, (0, 0, 0)),
                     _floats(o.get("rpy") if o is not None else None, (0, 0, 0)))


def _urdf_root(source):
    """A URDF as XML text (/robot_description) or a file path."""
    return ET.fromstring(source) if source.lstrip().startswith("<") else ET.parse(source).getroot()


def _link_transforms(root):
    parent = {j.find("child").get("link"): (j.find("parent").get("link"), _origin(j)) for j in root.findall("joint")}

    def link_transform(name):
        t = np.eye(4)
        while name in parent:
            name, tj = parent[name]
            t = tj @ t
        return t
    return link_transform


def urdf_frames(source):
    """Every link's 4x4 pose in the URDF's root (base_link)."""
    root = _urdf_root(source)
    link_transform = _link_transforms(root)
    return {link.get("name"): link_transform(link.get("name")) for link in root.findall("link")}


def load_urdf_model(source, package_dirs):
    """The URDF's mesh visuals in base_link coordinates: a list of (triangles (n, 3, 3), rgba)."""
    root = _urdf_root(source)
    materials = {}
    for m in root.findall("material"):
        color = m.find("color")
        if color is not None:
            materials[m.get("name")] = _floats(color.get("rgba"), (0.5, 0.5, 0.5, 1.0))
    link_transform = _link_transforms(root)

    def resolve(uri):
        if uri.startswith("package://"):
            package, rest = uri[len("package://"):].split("/", 1)
            return os.path.join(package_dirs[package], rest)
        return uri[len("file://"):] if uri.startswith("file://") else uri

    parts = []
    for link in root.findall("link"):
        tl = link_transform(link.get("name"))
        for vis in link.findall("visual"):
            mesh = vis.find("geometry/mesh")
            if mesh is None:
                continue
            mat = vis.find("material")
            rgba = (0.5, 0.5, 0.5, 1.0)
            if mat is not None:
                inline = mat.find("color")
                rgba = _floats(inline.get("rgba"), rgba) if inline is not None else materials.get(mat.get("name"), rgba)
            tri = read_stl(resolve(mesh.get("filename"))) * np.asarray(_floats(mesh.get("scale"), (1, 1, 1)), np.float32)
            t = tl @ _origin(vis)
            v = tri.reshape(-1, 3) @ t[:3, :3].T.astype(np.float32) + t[:3, 3].astype(np.float32)
            parts.append((v.reshape(-1, 3, 3), tuple(rgba), vis.get("name") or link.get("name")))
    return parts


def rotor_centres(parts, pattern="motor"):
    """The top centre of every visual whose name contains pattern, in base_link: where the
    rotors spin (the URDF has motors but no propellers)."""
    centres = []
    for tri, _, name in parts:
        if pattern in name and len(tri):
            v = tri.reshape(-1, 3)
            lo, hi = v.min(axis=0), v.max(axis=0)
            centres.append(((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, hi[2]))
    return np.array(centres, float).reshape(-1, 3)


def mesh_vertices(parts):
    """Interleaved (n, 9) float32 vertices: position, flat face normal, colour."""
    blocks = []
    for tri, rgba, *_ in parts:
        if not len(tri):
            continue
        n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
        n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
        block = np.empty((len(tri), 3, 9), np.float32)
        block[:, :, 0:3] = tri
        block[:, :, 3:6] = n[:, None, :]
        block[:, :, 6:9] = rgba[:3]
        blocks.append(block.reshape(-1, 9))
    return np.concatenate(blocks) if blocks else np.zeros((0, 9), np.float32)


def xyzi(message):
    """(n, 4) float32 x, y, z, intensity of a PointCloud2 with float32 fields of those names."""
    offsets = {f.name: f.offset for f in message.fields}
    names = [k for k in ("x", "y", "z", "intensity") if k in offsets]
    dtype = np.dtype({"names": names, "formats": ["<f4"] * len(names),
                      "offsets": [offsets[k] for k in names], "itemsize": message.point_step})
    raw = np.frombuffer(bytes(message.data), dtype=dtype, count=message.width * message.height)
    out = np.zeros((len(raw), 4), np.float32)
    for i, k in enumerate(names):
        out[:, i] = raw[k]
    return out


# ----- the overlay --------------------------------------------------------------------

def hud_image(width, height, lines, legend, footer, font_path, size=19, warning="", insets=(0, 0, 0, 0)):
    """RGBA (height, width, 4) uint8 on transparent. Everything sits inside the insets (top,
    right, bottom, left; px), the bands the ground station's own overlays cover (QGC: the
    toolbar, the tool strip, the camera panel, the instruments and the map thumbnail): the
    text lines and the footer on a translucent panel top left, a colour legend top right, a
    warning centred low."""
    from PIL import Image, ImageDraw, ImageFont
    img = Image.new("RGBA", (width, height), (0, 0, 0, 0))
    draw = ImageDraw.Draw(img)
    try:
        bold = ImageFont.truetype(font_path.replace(".ttf", "-Bold.ttf"), size + 1)
        font = ImageFont.truetype(font_path, size)
        small = ImageFont.truetype(font_path, size - 4)
    except OSError:
        bold = font = small = ImageFont.load_default()
    top, right, bottom, left = (int(v) for v in insets)
    pad, step = 14, size + 9
    x0, y0 = left + 12, top + 12
    widths = [draw.textlength(t, font=bold if i == 0 else font) for i, t in enumerate(lines)]
    widths.append(draw.textlength(footer, font=small))
    draw.rounded_rectangle((x0, y0, x0 + 2 * pad + max(widths), y0 + pad + step * len(lines) + size),
                           radius=10, fill=(4, 8, 14, 175))
    y = y0 + pad - 2
    for i, text in enumerate(lines):
        draw.text((x0 + pad, y), text, font=bold if i == 0 else font,
                  fill=(236, 242, 248, 255) if i == 0 else (176, 198, 214, 255))
        y += step
    draw.text((x0 + pad, y + 1), footer, font=small, fill=(150, 165, 180, 230))
    x, ly = width - right - 26, top + 22
    if legend:                                     # its own panel: it sits over the brightest returns
        span = sum(draw.textlength(label, font=small) + 36 for label, _ in legend) - 18
        draw.rounded_rectangle((x - span - 10, y0, width - right - 12, y0 + 34), radius=10, fill=(4, 8, 14, 175))
    for label, rgb in reversed(legend):
        w = draw.textlength(label, font=small)
        x -= w
        draw.text((x, ly), label, font=small, fill=(200, 210, 220, 255))
        x -= 18
        draw.ellipse((x, ly + 3, x + 11, ly + 14), fill=tuple(int(255 * c) for c in rgb) + (255,))
        x -= 18
    if warning:
        w, wy = draw.textlength(warning, font=bold), height - bottom - 48
        draw.rounded_rectangle(((width - w) / 2 - 14, wy, (width + w) / 2 + 14, wy + 36),
                               radius=8, fill=(60, 30, 0, 190))
        draw.text(((width - w) / 2, wy + 4), warning, font=bold, fill=(255, 190, 80, 255))
    return np.asarray(img, np.uint8)


# ----- the GPU renderer ---------------------------------------------------------------

POINTS_VS = """
#version 430 core
layout(location = 0) in vec3 pos;
layout(location = 1) in float intensity;
uniform mat4 vp;
uniform float size_world;
uniform float focal_px;
uniform vec2 size_px;
uniform vec2 zrange;
uniform float clip_z;
uniform vec3 cut_eye;
uniform vec3 cut_target;
uniform vec2 cut_radius;     // at the camera, at the aircraft
uniform float cut_end;       // fraction of the way to the aircraft where the cut stops
uniform float intensity_ref; // reflectivity that shades fully bright
uniform int mode;            // 0: colour by height, shaded by intensity; 1: flat colour
uniform vec3 flat_color;
uniform float pull;          // fraction of the way to the camera: live scans lie on map surfaces
out vec3 v_color;
out float v_depth;
vec3 turbo(float x) {        // Google's polynomial approximation of the Turbo colormap
    const vec4 kr4 = vec4(0.13572138, 4.61539260, -42.66032258, 132.13108234);
    const vec4 kg4 = vec4(0.09140261, 2.19418839, 4.84296658, -14.18503333);
    const vec4 kb4 = vec4(0.10667330, 12.64194608, -60.58204836, 110.36276771);
    const vec2 kr2 = vec2(-152.94239396, 59.28637943);
    const vec2 kg2 = vec2(4.27729857, 2.82956604);
    const vec2 kb2 = vec2(-89.90310912, 27.34824973);
    x = clamp(x, 0.0, 1.0);
    vec4 v4 = vec4(1.0, x, x * x, x * x * x);
    vec2 v2 = v4.zw * v4.z;
    return vec3(dot(v4, kr4) + dot(v2, kr2), dot(v4, kg4) + dot(v2, kg2), dot(v4, kb4) + dot(v2, kb2));
}
bool hidden(vec3 p) {
    if (p.z > clip_z) return true;                       // the dollhouse cut while enclosed
    vec3 d = cut_target - cut_eye;                       // the cone from the camera to the aircraft
    float t = dot(p - cut_eye, d) / dot(d, d);
    if (t <= 0.0 || t >= cut_end) return false;
    return distance(p, cut_eye + t * d) < mix(cut_radius.x, cut_radius.y, t);
}
void main() {
    if (hidden(pos)) {
        gl_Position = vec4(2.0, 2.0, 2.0, 1.0);          // outside the clip volume: dropped
        gl_PointSize = 1.0;
        v_color = vec3(0.0);
        v_depth = 0.0;
        return;
    }
    vec4 clip = vp * vec4(mix(pos, cut_eye, pull), 1.0);
    gl_Position = clip;
    v_depth = clip.w;
    gl_PointSize = clamp(size_world * focal_px / max(clip.w, 0.05), size_px.x, size_px.y);
    if (mode == 0) {
        float shade = 0.62 + 0.38 * clamp(intensity / intensity_ref, 0.0, 1.0);
        v_color = turbo(0.10 + 0.85 * (pos.z - zrange.x) / (zrange.y - zrange.x)) * shade;
    } else {
        v_color = flat_color;
    }
}
"""

POINTS_FS = """
#version 430 core
in vec3 v_color;
in float v_depth;
layout(location = 0) out vec4 color;
layout(location = 1) out float depth;
void main() {
    vec2 c = gl_PointCoord * 2.0 - 1.0;
    if (dot(c, c) > 1.0) discard;
    color = vec4(v_color, 1.0);
    depth = v_depth;
}
"""

MESH_VS = """
#version 430 core
layout(location = 0) in vec3 pos;
layout(location = 1) in vec3 normal;
layout(location = 2) in vec3 albedo;
uniform mat4 vp;
uniform mat4 model;
out vec3 v_normal;
out vec3 v_albedo;
out vec3 v_world;
out float v_depth;
void main() {
    vec4 world = model * vec4(pos, 1.0);
    vec4 clip = vp * world;
    gl_Position = clip;
    v_world = world.xyz;
    v_depth = clip.w;
    v_normal = mat3(model) * normal;
    v_albedo = albedo;
}
"""

MESH_FS = """
#version 430 core
in vec3 v_normal;
in vec3 v_albedo;
in vec3 v_world;
in float v_depth;
uniform vec3 eye;
uniform vec3 light_dir;      // the direction the light travels
uniform float lift;          // albedo pulled toward light grey: the airframe is near-black carbon
layout(location = 0) out vec4 color;
layout(location = 1) out float depth;
void main() {
    vec3 n = normalize(v_normal);
    vec3 view = normalize(eye - v_world);
    if (dot(n, view) < 0.0) n = -n;                      // STL winding is not to be trusted
    vec3 base = mix(v_albedo, vec3(0.80, 0.76, 0.68), lift);
    float key = max(dot(n, -light_dir), 0.0);
    float fill = max(dot(n, normalize(vec3(-0.4, 0.6, 0.3))), 0.0);
    float rim = pow(1.0 - max(dot(n, view), 0.0), 2.5);
    vec3 c = base * (0.32 + 0.85 * key + 0.25 * fill) + vec3(0.35, 0.65, 0.90) * rim * 0.8;
    color = vec4(c, 1.0);
    depth = v_depth;
}
"""

LINE_VS = """
#version 430 core
layout(location = 0) in vec3 pos;
layout(location = 1) in vec3 col;
uniform mat4 vp;
uniform vec3 eye;
uniform float pull;          // as the live scans: a footprint drawn on the ground stays on top of it
out vec3 v_color;
out float v_depth;
void main() {
    vec4 clip = vp * vec4(mix(pos, eye, pull), 1.0);
    gl_Position = clip;
    v_depth = clip.w;
    v_color = col;
}
"""

LINE_FS = """
#version 430 core
in vec3 v_color;
in float v_depth;
layout(location = 0) out vec4 color;
layout(location = 1) out float depth;
void main() { color = vec4(v_color, 1.0); depth = v_depth; }
"""

FULLSCREEN_VS = """
#version 430 core
out vec2 uv;
void main() {                // one triangle covering the screen
    vec2 p = vec2((gl_VertexID << 1) & 2, gl_VertexID & 2);
    uv = p;
    gl_Position = vec4(p * 2.0 - 1.0, 0.0, 1.0);
}
"""

EDL_FS = """
#version 430 core
in vec2 uv;
uniform sampler2D color_tex;
uniform sampler2D depth_tex;
uniform vec2 texel;
uniform float strength;
uniform float radius;
uniform vec3 sky_top;
uniform vec3 sky_bottom;
out vec4 frag;
void main() {                // eye-dome lighting: darken where a pixel lies behind its neighbours
    float d = texture(depth_tex, uv).r;
    if (d <= 0.0) { frag = vec4(mix(sky_bottom, sky_top, uv.y), 1.0); return; }
    float ld = log2(d);
    float sum = 0.0;
    for (int i = 0; i < 8; i++) {
        float a = 0.7853982 * float(i);
        float dn = texture(depth_tex, uv + vec2(cos(a), sin(a)) * texel * radius).r;
        if (dn > 0.0) sum += max(0.0, ld - log2(dn));
    }
    frag = vec4(texture(color_tex, uv).rgb * exp(-strength * sum), 1.0);
}
"""

HUD_FS = """
#version 430 core
in vec2 uv;
uniform sampler2D hud_tex;
out vec4 frag;
void main() { frag = texture(hud_tex, vec2(uv.x, 1.0 - uv.y)); }
"""


class Renderer:
    """Headless OpenGL 4.3 core on EGL: points, the URDF mesh and lines into a colour +
    linear-depth target, eye-dome lighting and the overlay into the output, read back RGBA
    through two pixel-pack buffers (each call returns the previous call's frame, so the
    copy never waits on the GPU)."""

    def __init__(self, width, height, map_capacity, scan_capacity, mesh, trail_capacity, overlay_capacity=8192):
        os.environ.setdefault("PYOPENGL_PLATFORM", "egl")
        from OpenGL import EGL, GL
        self.GL, self.EGL = GL, EGL
        self.width, self.height = width, height
        self.map_capacity, self.scan_capacity = map_capacity, scan_capacity
        self.trail_capacity, self.overlay_capacity = trail_capacity, overlay_capacity
        self._egl_context()
        self.points_prog = self._compile(POINTS_VS, POINTS_FS)
        self.mesh_prog = self._compile(MESH_VS, MESH_FS)
        self.line_prog = self._compile(LINE_VS, LINE_FS)
        self.edl_prog = self._compile(FULLSCREEN_VS, EDL_FS)
        self.hud_prog = self._compile(FULLSCREEN_VS, HUD_FS)
        self._uniforms = {}
        self._targets()
        self.map_vao, self.map_vbo = self._buffer(map_capacity, ((0, 3, 0), (1, 1, 12)), 16)
        self.scan_vao, self.scan_vbo = self._buffer(scan_capacity, ((0, 3, 0), (1, 1, 12)), 16)
        self.e1r_vao, self.e1r_vbo = self._buffer(scan_capacity, ((0, 3, 0), (1, 1, 12)), 16)
        self.trail_vao, self.trail_vbo = self._buffer(trail_capacity, ((0, 3, 0),), 12)
        self.overlay_vao, self.overlay_vbo = self._buffer(overlay_capacity, ((0, 3, 0), (1, 3, 12)), 24)
        mesh = np.ascontiguousarray(mesh, np.float32)
        self.mesh_vao, self.mesh_vbo = self._buffer(max(len(mesh), 1), ((0, 3, 0), (1, 3, 12), (2, 3, 24)), 36)
        if len(mesh):
            GL.glBindBuffer(GL.GL_ARRAY_BUFFER, self.mesh_vbo)
            GL.glBufferSubData(GL.GL_ARRAY_BUFFER, 0, mesh.nbytes, mesh)
        self.mesh_count = len(mesh)
        self.empty_vao = GL.glGenVertexArrays(1)
        self.map_count = self.scan_count = self.e1r_count = self.trail_count = self.overlay_count = 0
        self.hud_tex = self._texture(GL.GL_RGBA8, GL.GL_RGBA, GL.GL_UNSIGNED_BYTE, linear=True)
        self.pbos = GL.glGenBuffers(2)
        for pbo in self.pbos:
            GL.glBindBuffer(GL.GL_PIXEL_PACK_BUFFER, pbo)
            GL.glBufferData(GL.GL_PIXEL_PACK_BUFFER, width * height * 4, None, GL.GL_STREAM_READ)
        GL.glBindBuffer(GL.GL_PIXEL_PACK_BUFFER, 0)
        self.frame_index = 0
        self.gpu = GL.glGetString(GL.GL_RENDERER).decode()

    def _egl_context(self):
        EGL = self.EGL
        self.display = EGL.eglGetDisplay(EGL.EGL_DEFAULT_DISPLAY)
        major, minor = EGL.EGLint(), EGL.EGLint()
        if not EGL.eglInitialize(self.display, ctypes.pointer(major), ctypes.pointer(minor)):
            raise RuntimeError("eglInitialize failed")
        attrs = (EGL.EGLint * 13)(EGL.EGL_SURFACE_TYPE, EGL.EGL_PBUFFER_BIT, EGL.EGL_RED_SIZE, 8,
                                  EGL.EGL_GREEN_SIZE, 8, EGL.EGL_BLUE_SIZE, 8, EGL.EGL_DEPTH_SIZE, 24,
                                  EGL.EGL_RENDERABLE_TYPE, EGL.EGL_OPENGL_BIT, EGL.EGL_NONE)
        config, count = EGL.EGLConfig(), EGL.EGLint()
        if not EGL.eglChooseConfig(self.display, attrs, ctypes.pointer(config), 1, ctypes.pointer(count)) or not count.value:
            raise RuntimeError("no EGL config for desktop OpenGL")
        self.surface = EGL.eglCreatePbufferSurface(self.display, config,
                                                   (EGL.EGLint * 5)(EGL.EGL_WIDTH, 16, EGL.EGL_HEIGHT, 16, EGL.EGL_NONE))
        EGL.eglBindAPI(EGL.EGL_OPENGL_API)
        # EGL_CONTEXT_MAJOR_VERSION 4, EGL_CONTEXT_MINOR_VERSION 3, EGL_CONTEXT_OPENGL_PROFILE_MASK core
        self.context = EGL.eglCreateContext(self.display, config, EGL.EGL_NO_CONTEXT,
                                            (EGL.EGLint * 7)(0x3098, 4, 0x30FB, 3, 0x30FD, 1, EGL.EGL_NONE))
        if not self.context:
            raise RuntimeError("cannot create an OpenGL 4.3 core context")
        EGL.eglMakeCurrent(self.display, self.surface, self.surface, self.context)

    def _compile(self, vs, fs):
        GL = self.GL
        program = GL.glCreateProgram()
        for source, kind in ((vs, GL.GL_VERTEX_SHADER), (fs, GL.GL_FRAGMENT_SHADER)):
            shader = GL.glCreateShader(kind)
            GL.glShaderSource(shader, source)
            GL.glCompileShader(shader)
            if not GL.glGetShaderiv(shader, GL.GL_COMPILE_STATUS):
                raise RuntimeError(GL.glGetShaderInfoLog(shader).decode())
            GL.glAttachShader(program, shader)
        GL.glLinkProgram(program)
        if not GL.glGetProgramiv(program, GL.GL_LINK_STATUS):
            raise RuntimeError(GL.glGetProgramInfoLog(program).decode())
        return program

    def _u(self, program, name):
        key = (program, name)
        if key not in self._uniforms:
            self._uniforms[key] = self.GL.glGetUniformLocation(program, name)
        return self._uniforms[key]

    def _texture(self, internal, fmt, kind, linear=False):
        GL = self.GL
        tex = GL.glGenTextures(1)
        GL.glBindTexture(GL.GL_TEXTURE_2D, tex)
        GL.glTexImage2D(GL.GL_TEXTURE_2D, 0, internal, self.width, self.height, 0, fmt, kind, None)
        filt = GL.GL_LINEAR if linear else GL.GL_NEAREST
        GL.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_MIN_FILTER, filt)
        GL.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_MAG_FILTER, filt)
        GL.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_WRAP_S, GL.GL_CLAMP_TO_EDGE)
        GL.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_WRAP_T, GL.GL_CLAMP_TO_EDGE)
        return tex

    def _targets(self):
        GL = self.GL
        self.color_tex = self._texture(GL.GL_RGBA8, GL.GL_RGBA, GL.GL_UNSIGNED_BYTE)
        self.depth_tex = self._texture(GL.GL_R32F, GL.GL_RED, GL.GL_FLOAT)
        self.scene_fbo = GL.glGenFramebuffers(1)
        GL.glBindFramebuffer(GL.GL_FRAMEBUFFER, self.scene_fbo)
        GL.glFramebufferTexture2D(GL.GL_FRAMEBUFFER, GL.GL_COLOR_ATTACHMENT0, GL.GL_TEXTURE_2D, self.color_tex, 0)
        GL.glFramebufferTexture2D(GL.GL_FRAMEBUFFER, GL.GL_COLOR_ATTACHMENT1, GL.GL_TEXTURE_2D, self.depth_tex, 0)
        depth_rb = GL.glGenRenderbuffers(1)
        GL.glBindRenderbuffer(GL.GL_RENDERBUFFER, depth_rb)
        GL.glRenderbufferStorage(GL.GL_RENDERBUFFER, GL.GL_DEPTH_COMPONENT24, self.width, self.height)
        GL.glFramebufferRenderbuffer(GL.GL_FRAMEBUFFER, GL.GL_DEPTH_ATTACHMENT, GL.GL_RENDERBUFFER, depth_rb)
        GL.glDrawBuffers(2, [GL.GL_COLOR_ATTACHMENT0, GL.GL_COLOR_ATTACHMENT1])
        if GL.glCheckFramebufferStatus(GL.GL_FRAMEBUFFER) != GL.GL_FRAMEBUFFER_COMPLETE:
            raise RuntimeError("scene framebuffer incomplete")
        self.out_fbo = GL.glGenFramebuffers(1)
        GL.glBindFramebuffer(GL.GL_FRAMEBUFFER, self.out_fbo)
        out_rb = GL.glGenRenderbuffers(1)
        GL.glBindRenderbuffer(GL.GL_RENDERBUFFER, out_rb)
        GL.glRenderbufferStorage(GL.GL_RENDERBUFFER, GL.GL_RGBA8, self.width, self.height)
        GL.glFramebufferRenderbuffer(GL.GL_FRAMEBUFFER, GL.GL_COLOR_ATTACHMENT0, GL.GL_RENDERBUFFER, out_rb)
        if GL.glCheckFramebufferStatus(GL.GL_FRAMEBUFFER) != GL.GL_FRAMEBUFFER_COMPLETE:
            raise RuntimeError("output framebuffer incomplete")

    def _buffer(self, capacity, attributes, stride):
        """A VAO + VBO of capacity vertices; attributes: (location, floats, byte offset)."""
        GL = self.GL
        vao, vbo = GL.glGenVertexArrays(1), GL.glGenBuffers(1)
        GL.glBindVertexArray(vao)
        GL.glBindBuffer(GL.GL_ARRAY_BUFFER, vbo)
        GL.glBufferData(GL.GL_ARRAY_BUFFER, capacity * stride, None, GL.GL_DYNAMIC_DRAW)
        for location, size, offset in attributes:
            GL.glEnableVertexAttribArray(location)
            GL.glVertexAttribPointer(location, size, GL.GL_FLOAT, GL.GL_FALSE, stride, ctypes.c_void_p(offset))
        GL.glBindVertexArray(0)
        return vao, vbo

    def append_map(self, points):
        n = min(len(points), self.map_capacity - self.map_count)
        if n <= 0:
            return 0
        GL = self.GL
        GL.glBindBuffer(GL.GL_ARRAY_BUFFER, self.map_vbo)
        GL.glBufferSubData(GL.GL_ARRAY_BUFFER, self.map_count * 16, n * 16, np.ascontiguousarray(points[:n], np.float32))
        self.map_count += n
        return n

    def clear_map(self):
        self.map_count = 0

    def _replace(self, vbo, points, capacity):
        n = min(len(points), capacity)
        if n:
            GL = self.GL
            data = np.ascontiguousarray(points[:n], np.float32)
            GL.glBindBuffer(GL.GL_ARRAY_BUFFER, vbo)
            GL.glBufferSubData(GL.GL_ARRAY_BUFFER, 0, data.nbytes, data)
        return n

    def set_scan(self, points):
        self.scan_count = self._replace(self.scan_vbo, points, self.scan_capacity) if points is not None else 0

    def set_e1r(self, points):
        self.e1r_count = self._replace(self.e1r_vbo, points, self.scan_capacity) if points is not None else 0

    def set_trail(self, points):
        self.trail_count = self._replace(self.trail_vbo, points, self.trail_capacity)

    def set_overlay(self, vertices):
        """(n, 6) position + colour, a line list."""
        self.overlay_count = self._replace(self.overlay_vbo, vertices, self.overlay_capacity) // 2 * 2

    def set_hud(self, rgba):
        GL = self.GL
        GL.glBindTexture(GL.GL_TEXTURE_2D, self.hud_tex)
        GL.glTexSubImage2D(GL.GL_TEXTURE_2D, 0, 0, 0, self.width, self.height, GL.GL_RGBA, GL.GL_UNSIGNED_BYTE,
                           np.ascontiguousarray(rgba))

    def _draw_points(self, vao, count, frame, size_world, size_px, mode, color=(1.0, 1.0, 1.0), pull=0.0):
        if not count:
            return
        GL, p = self.GL, self.points_prog
        GL.glUseProgram(p)
        GL.glUniform1f(self._u(p, "pull"), pull)
        GL.glUniformMatrix4fv(self._u(p, "vp"), 1, GL.GL_TRUE, frame["vp"])
        GL.glUniform1f(self._u(p, "size_world"), size_world)
        GL.glUniform1f(self._u(p, "focal_px"), frame["focal"])
        GL.glUniform2f(self._u(p, "size_px"), *size_px)
        GL.glUniform2f(self._u(p, "zrange"), *frame["zrange"])
        GL.glUniform1f(self._u(p, "clip_z"), frame["clip_z"])
        GL.glUniform3f(self._u(p, "cut_eye"), *frame["eye"])
        GL.glUniform3f(self._u(p, "cut_target"), *frame["cut_target"])
        GL.glUniform2f(self._u(p, "cut_radius"), *frame["cut_radius"])
        GL.glUniform1f(self._u(p, "cut_end"), frame["cut_end"])
        GL.glUniform1f(self._u(p, "intensity_ref"), frame["intensity_ref"])
        GL.glUniform1i(self._u(p, "mode"), mode)
        GL.glUniform3f(self._u(p, "flat_color"), *color)
        GL.glBindVertexArray(vao)
        GL.glDrawArrays(GL.GL_POINTS, 0, count)

    def render(self, frame, style):
        """frame: vp (4x4 float32), eye, focal, zrange, clip_z, cut_target, cut_radius,
        cut_end, intensity_ref, model (4x4 or None). Returns the previous frame's RGBA rows
        (bottom first) as bytes, or None on the first call."""
        GL = self.GL
        GL.glBindFramebuffer(GL.GL_FRAMEBUFFER, self.scene_fbo)
        GL.glViewport(0, 0, self.width, self.height)
        GL.glClearBufferfv(GL.GL_COLOR, 0, [0.0, 0.0, 0.0, 0.0])
        GL.glClearBufferfv(GL.GL_COLOR, 1, [0.0, 0.0, 0.0, 0.0])
        GL.glClear(GL.GL_DEPTH_BUFFER_BIT)
        GL.glEnable(GL.GL_DEPTH_TEST)
        GL.glEnable(GL.GL_PROGRAM_POINT_SIZE)
        self._draw_points(self.map_vao, self.map_count, frame, style["map_size"], style["map_px"], 0)
        self._draw_points(self.e1r_vao, self.e1r_count, frame, style["scan_size"], style["scan_px"], 1,
                          style["e1r_color"], style["pull"])
        self._draw_points(self.scan_vao, self.scan_count, frame, style["scan_size"], style["scan_px"], 1,
                          style["avia_color"], style["pull"])
        p = self.line_prog
        GL.glUseProgram(p)
        GL.glUniformMatrix4fv(self._u(p, "vp"), 1, GL.GL_TRUE, frame["vp"])
        GL.glUniform3f(self._u(p, "eye"), *frame["eye"])
        GL.glUniform1f(self._u(p, "pull"), style["pull"])
        try:
            GL.glLineWidth(style["line_px"])
        except GL.GLError:
            pass
        if self.trail_count > 1:
            GL.glVertexAttrib3f(1, *style["trail_color"])             # attribute 1 is off in the trail's VAO
            GL.glBindVertexArray(self.trail_vao)
            GL.glDrawArrays(GL.GL_LINE_STRIP, 0, self.trail_count)
        if self.overlay_count:
            GL.glBindVertexArray(self.overlay_vao)
            GL.glDrawArrays(GL.GL_LINES, 0, self.overlay_count)
        if self.mesh_count and frame["model"] is not None:
            p = self.mesh_prog
            GL.glUseProgram(p)
            GL.glUniformMatrix4fv(self._u(p, "vp"), 1, GL.GL_TRUE, frame["vp"])
            GL.glUniformMatrix4fv(self._u(p, "model"), 1, GL.GL_TRUE, frame["model"].astype(np.float32))
            GL.glUniform3f(self._u(p, "eye"), *frame["eye"])
            light = np.array(style["light_dir"], float)
            GL.glUniform3f(self._u(p, "light_dir"), *(light / np.linalg.norm(light)))
            GL.glUniform1f(self._u(p, "lift"), style["mesh_lift"])
            GL.glBindVertexArray(self.mesh_vao)
            GL.glDrawArrays(GL.GL_TRIANGLES, 0, self.mesh_count)
        # Eye-dome lighting into the output, then the overlay.
        GL.glDisable(GL.GL_DEPTH_TEST)
        GL.glBindFramebuffer(GL.GL_FRAMEBUFFER, self.out_fbo)
        p = self.edl_prog
        GL.glUseProgram(p)
        GL.glActiveTexture(GL.GL_TEXTURE0)
        GL.glBindTexture(GL.GL_TEXTURE_2D, self.color_tex)
        GL.glActiveTexture(GL.GL_TEXTURE1)
        GL.glBindTexture(GL.GL_TEXTURE_2D, self.depth_tex)
        GL.glUniform1i(self._u(p, "color_tex"), 0)
        GL.glUniform1i(self._u(p, "depth_tex"), 1)
        GL.glUniform2f(self._u(p, "texel"), 1.0 / self.width, 1.0 / self.height)
        GL.glUniform1f(self._u(p, "strength"), style["edl_strength"])
        GL.glUniform1f(self._u(p, "radius"), style["edl_radius"])
        GL.glUniform3f(self._u(p, "sky_top"), *style["sky_top"])
        GL.glUniform3f(self._u(p, "sky_bottom"), *style["sky_bottom"])
        GL.glBindVertexArray(self.empty_vao)
        GL.glDrawArrays(GL.GL_TRIANGLES, 0, 3)
        GL.glEnable(GL.GL_BLEND)
        GL.glBlendFunc(GL.GL_SRC_ALPHA, GL.GL_ONE_MINUS_SRC_ALPHA)
        p = self.hud_prog
        GL.glUseProgram(p)
        GL.glActiveTexture(GL.GL_TEXTURE0)
        GL.glBindTexture(GL.GL_TEXTURE_2D, self.hud_tex)
        GL.glUniform1i(self._u(p, "hud_tex"), 0)
        GL.glDrawArrays(GL.GL_TRIANGLES, 0, 3)
        GL.glDisable(GL.GL_BLEND)
        # Start this frame's read-back; collect the previous one.
        GL.glReadBuffer(GL.GL_COLOR_ATTACHMENT0)
        GL.glBindBuffer(GL.GL_PIXEL_PACK_BUFFER, self.pbos[self.frame_index % 2])
        GL.glReadPixels(0, 0, self.width, self.height, GL.GL_RGBA, GL.GL_UNSIGNED_BYTE, ctypes.c_void_p(0))
        self.frame_index += 1
        if self.frame_index < 2:
            GL.glBindBuffer(GL.GL_PIXEL_PACK_BUFFER, 0)
            return None
        GL.glBindBuffer(GL.GL_PIXEL_PACK_BUFFER, self.pbos[self.frame_index % 2])
        size = self.width * self.height * 4
        ptr = GL.glMapBufferRange(GL.GL_PIXEL_PACK_BUFFER, 0, size, GL.GL_MAP_READ_BIT)
        pixels = ctypes.string_at(ptr, size)
        GL.glUnmapBuffer(GL.GL_PIXEL_PACK_BUFFER)
        GL.glBindBuffer(GL.GL_PIXEL_PACK_BUFFER, 0)
        return pixels

    def reset_readback(self):
        self.frame_index = 0

    def close(self):
        """Release the context and the display: exiting with them alive aborts the process."""
        EGL = self.EGL
        try:
            EGL.eglMakeCurrent(self.display, EGL.EGL_NO_SURFACE, EGL.EGL_NO_SURFACE, EGL.EGL_NO_CONTEXT)
            EGL.eglDestroyContext(self.display, self.context)
            EGL.eglDestroySurface(self.display, self.surface)
            EGL.eglTerminate(self.display)
        except Exception:
            pass


# ----- RTSP ---------------------------------------------------------------------------

class RtspStream:
    """An on-demand RTSP mount fed with RGBA frames: nvvidconv converts to NV12 and flips
    (OpenGL rows run bottom to top) on the VIC, NVENC encodes. Shared between clients like
    the camera's RGB stream; every PLAY forces a keyframe so a new receiver starts at once.
    `active` while a client's media is prepared."""

    def __init__(self, bind, port, mount, width, height, fps, bitrate, codec="h265"):
        import gi
        gi.require_version("Gst", "1.0")
        gi.require_version("GstRtspServer", "1.0")
        from gi.repository import GLib, Gst, GstRtspServer
        self.Gst = Gst
        Gst.init(None)
        for name in ("appsrc", "nvvidconv", f"nvv4l2{codec}enc", f"{codec}parse", f"rtp{codec}pay"):
            if not Gst.ElementFactory.find(name):
                raise RuntimeError(f"required GStreamer plugin unavailable: {name}")
        profile = "0" if codec == "h265" else "4"
        self._lock = threading.Lock()
        self._appsrc = self._encoder = self._media = None
        self.clients = 0
        self.frames_pushed = 0
        self._loop = GLib.MainLoop()
        self._server = GstRtspServer.RTSPServer.new()
        self._server.set_address(bind)
        self._server.set_service(str(port))
        self._server.connect("client-connected", self._client_connected)
        factory = GstRtspServer.RTSPMediaFactory.new()
        factory.set_shared(True)
        factory.set_eos_shutdown(False)
        factory.set_suspend_mode(GstRtspServer.RTSPSuspendMode.NONE)
        factory.set_launch(
            f"( appsrc name=frames is-live=true format=time do-timestamp=true block=false "
            f"max-buffers=2 leaky-type=downstream "
            f"caps=video/x-raw,format=RGBA,width={width},height={height},framerate={fps}/1 "
            f"! nvvidconv flip-method=6 ! video/x-raw(memory:NVMM),format=NV12 "
            f"! nvv4l2{codec}enc name=encoder bitrate={bitrate} control-rate=1 num-B-Frames=0 "
            f"iframeinterval={fps} idrinterval={fps} insert-sps-pps=true preset-level=1 profile={profile} "
            f"! {codec}parse config-interval=-1 "
            f"! rtp{codec}pay name=pay0 pt=96 config-interval=-1 mtu=1200 )")
        factory.connect("media-configure", self._configure)
        self._server.get_mount_points().add_factory(mount, factory)
        if not self._server.attach(None):
            raise RuntimeError(f"cannot bind RTSP {bind}:{port} (address or port in use)")
        threading.Thread(target=self._loop.run, daemon=True, name="lidar-view-glib").start()

    def _client_connected(self, _server, client):
        with self._lock:
            self.clients += 1
        client.connect("closed", self._client_closed)
        client.connect("play-request", self._play)

    def _client_closed(self, _client):
        with self._lock:
            self.clients = max(0, self.clients - 1)

    def _play(self, _client, _context):
        self.request_keyframe()

    def request_keyframe(self):
        with self._lock:
            encoder = self._encoder
        if encoder is not None:
            try:
                encoder.emit("force-IDR")
            except Exception:
                pass

    def _configure(self, _factory, media):
        element = media.get_element()
        source = element.get_by_name("frames")
        source.set_property("min-latency", 0)
        with self._lock:
            self._appsrc, self._encoder, self._media = source, element.get_by_name("encoder"), media
        media.connect("unprepared", self._unprepared)

    def _unprepared(self, media):
        with self._lock:
            if self._media is media:
                self._appsrc = self._encoder = self._media = None

    @property
    def active(self):
        with self._lock:
            return self._appsrc is not None

    def push(self, pixels):
        with self._lock:
            source = self._appsrc
        if source is not None:
            source.emit("push-buffer", self.Gst.Buffer.new_wrapped(pixels))
            self.frames_pushed += 1

    def close(self):
        self._loop.quit()


# ----- the node -----------------------------------------------------------------------

STYLE = {
    "map_size": 0.065, "map_px": (1.6, 9.0),
    "scan_size": 0.08, "scan_px": (2.4, 7.0), "pull": 0.004,
    "avia_color": (0.93, 0.97, 1.0), "e1r_color": (1.0, 0.28, 0.92), "trail_color": (0.15, 0.85, 1.0),
    "avia_fov_color": (0.42, 0.48, 0.55), "e1r_fov_color": (0.55, 0.18, 0.52), "plumb_color": (0.95, 0.78, 0.35),
    "rotor_color": (0.80, 0.86, 0.92),
    "line_px": 2.0,
    "light_dir": (0.35, 0.25, -1.0), "mesh_lift": 0.6,
    "edl_strength": 12.0, "edl_radius": 1.4,
    "intensity_ref": 80.0,
    "sky_top": (0.035, 0.050, 0.085), "sky_bottom": (0.0, 0.0, 0.0),
    "cut_radius": (0.25, 0.7), "cut_stop_m": 0.9,
}


class Scene:
    """What the ROS callbacks (one executor thread) hand to the render thread."""

    def __init__(self, voxel, map_capacity, trail_step, promote=3):
        self.lock = threading.Lock()
        self.dedup = VoxelDedup(voxel, map_capacity)        # ROS thread only
        self.promoter = Promoter(promote)                   # ROS thread only
        self.occupancy = Occupancy()                        # its own lock
        self.map_queue = collections.deque()
        self.map_clear = False
        self.scan = self.e1r = None
        self.scan_version = self.e1r_version = 0
        self.poses = PoseBuffer()
        self.trail = Trail(trail_step)
        self.sample = np.zeros((0, 3), np.float32)           # of the shown map, for the colour range
        self.scan_times = collections.deque(maxlen=30)
        self.e1r_times = collections.deque(maxlen=30)
        self.speed = 0.0
        self.start_z = None
        self.jumps = collections.deque(maxlen=20)           # when the pose last leapt (restart, divergence)
        self.health = None                                  # the lio watchdog's last verdict (a dict)

    def add_map(self, points):
        fresh = self.dedup.add(points)
        if not len(fresh):
            return
        self.occupancy.add(fresh[:, :3])
        shown = self.promoter.add(fresh)
        if not len(shown):
            return
        step = max(1, len(shown) // 2000)
        with self.lock:
            self.map_queue.append(shown)
            self.sample = np.concatenate([self.sample, shown[::step, :3]])[-60000:]

    def reset_map(self):
        self.dedup.clear()
        self.promoter.clear()
        self.occupancy.clear()
        with self.lock:
            self.map_queue.clear()
            self.map_clear = True
            self.sample = np.zeros((0, 3), np.float32)


def rate(times):
    return (len(times) - 1) / (times[-1] - times[0]) if len(times) > 2 and times[-1] > times[0] else 0.0


def create_node(options, scene, stream):
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import String, UInt32

    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)

    class LidarView(Node):
        def __init__(self):
            super().__init__("lidar_view")
            self.heavy = []                       # map, scan and E1R subscriptions, only while watched
            self.last_odom = None
            self.stats = {}
            # The model and every mount from the description robot_state_publisher serves.
            self.description, self.frames, self.imu_mount = None, None, None
            self.description_ready = threading.Event()
            self.create_subscription(String, options.description_topic, self._description, latched)
            self.create_subscription(Odometry, options.odometry_topic, self._odom, 50)
            # The lio watchdog's verdict (why the map stopped) and the map's epoch (when what
            # this view accumulated must go: points taken back, or a new map).
            self.map_epoch = None
            self.create_subscription(String, options.health_topic, self._health, latched)
            self.create_subscription(UInt32, options.map_topic_base + "/epoch", self._map_epoch, latched)
            self.create_timer(0.5, self._watch)
            self.diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
            self.create_timer(1.0, self._diagnose)

        def _description(self, message):
            frames = urdf_frames(message.data)
            missing = [f for f in (options.imu_frame, options.avia_frame, options.e1r_frame) if f not in frames]
            if missing:
                self.get_logger().error(f"{options.description_topic} lacks {', '.join(missing)}")
                return
            self.description, self.frames, self.imu_mount = message.data, frames, frames[options.imu_frame]
            if not self.description_ready.is_set():
                self.get_logger().info(f"model and mounts from {options.description_topic} ({len(frames)} links)")
            self.description_ready.set()

        def _odom(self, message):
            if self.imu_mount is None:
                return                            # no description yet
            p, q = message.pose.pose.position, message.pose.pose.orientation
            base, quat = base_from_imu((p.x, p.y, p.z), (q.x, q.y, q.z, q.w), self.imu_mount)
            t = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            newest = scene.poses.newest()
            if newest is not None and (t < newest[0] - 1.0 or t > newest[0] + 5.0 or
                                       np.linalg.norm(base - newest[1]) > max(25.0, 60.0 * abs(t - newest[0]))):
                # FAST-LIO restarted (a new origin; once) or is diverging (every pose): start over.
                with scene.lock:
                    scene.trail.clear()
                    scene.start_z, scene.speed = None, 0.0
                    scene.jumps.append(time.monotonic())
                scene.poses.clear()
            before = scene.poses.at(t - 0.5)
            if not scene.poses.add(t, base, quat, time.monotonic()):
                return
            with scene.lock:
                if before is not None:
                    scene.speed = 0.7 * scene.speed + 0.3 * float(np.linalg.norm(base - before[0]) / 0.5)
                if scene.start_z is None:
                    scene.start_z = float(base[2])
                scene.trail.add(base)
            self.last_odom = time.monotonic()

        def _health(self, message):
            try:
                verdict = json.loads(message.data)
            except ValueError:
                return
            if isinstance(verdict, dict) and verdict.get("state") in ("ok", "degraded", "diverged"):
                scene.health = verdict

        def _map_epoch(self, message):
            if self.map_epoch is not None and message.data != self.map_epoch and self.heavy:
                scene.reset_map()                 # the whole map follows on /updates
                self.get_logger().info(f"map epoch {message.data}: map reloaded")
            self.map_epoch = message.data

        def _watch(self):
            want = options.always_on or (stream is not None and stream.active)
            if want and not self.heavy:
                scene.reset_map()                 # lio_map sends the whole map to the new subscriber
                self.heavy = [
                    self.create_subscription(PointCloud2, options.map_topic, lambda m: scene.add_map(xyzi(m)), 10),
                    self.create_subscription(PointCloud2, options.scan_topic, self._scan, 5),
                    self.create_subscription(PointCloud2, options.e1r_topic, self._e1r, 5)]
                self.get_logger().info("viewer connected: rendering")
            elif not want and self.heavy:
                for sub in self.heavy:
                    self.destroy_subscription(sub)
                self.heavy = []
                scene.reset_map()
                with scene.lock:
                    scene.scan = scene.e1r = None
                    scene.scan_version += 1
                    scene.e1r_version += 1
                self.get_logger().info("no viewer: idle")

        def _scan(self, message):
            points = xyzi(message)
            with scene.lock:
                scene.scan, scene.scan_version = points, scene.scan_version + 1
                scene.scan_times.append(time.monotonic())

        def _e1r(self, message):
            points = xyzi(message)
            with scene.lock:
                scene.e1r, scene.e1r_version = points, scene.e1r_version + 1
                scene.e1r_times.append(time.monotonic())

        def _diagnose(self):
            watching = bool(self.heavy)
            fresh_pose = self.last_odom is not None and time.monotonic() - self.last_odom < 2.0
            level = DiagnosticStatus.OK if fresh_pose or not watching else DiagnosticStatus.WARN
            if not watching:
                text = "Idle (no viewer)"
            elif fresh_pose:
                text = f"Streaming at {self.stats.get('fps', 0.0):.1f} fps"
            else:
                text = "Streaming without FAST-LIO poses"
            values = {"clients": stream.clients if stream else 0, "rendering": watching,
                      "fps": round(self.stats.get("fps", 0.0), 1), "render_ms": round(self.stats.get("render_ms", 0.0), 1),
                      "map_points": self.stats.get("map_points", 0), "agl_m": round(self.stats.get("agl", 0.0), 2),
                      "ground": self.stats.get("ground", ""), "boom_m": round(self.stats.get("boom", 0.0), 1),
                      "enclosed": self.stats.get("enclosed", False),
                      "frames_pushed": stream.frames_pushed if stream else 0,
                      "gpu": self.stats.get("gpu", ""), "uri": options.advertised_uri}
            out = DiagnosticArray()
            out.header.stamp = self.get_clock().now().to_msg()
            out.status = [DiagnosticStatus(level=level, name="lidar_view", hardware_id="FAST-LIO map video",
                                           message=text, values=[KeyValue(key=k, value=str(v)) for k, v in values.items()])]
            self.diag_pub.publish(out)

    return LidarView()


def render_loop(options, scene, stream, node, stop, snapshot=None):
    parts = load_urdf_model(options.description or options.urdf, dict(pd.split("=", 1) for pd in options.package_dir))
    mesh = mesh_vertices(parts)
    renderer = Renderer(options.width, options.height, options.map_capacity, 400_000, mesh, scene.trail.cap)
    # Rotor discs as two rings per motor (decorative: the URDF has no propellers), in base_link.
    rotors = np.concatenate([ring_lines(r) + c for c in rotor_centres(parts) + (0, 0, 0.02)
                             for r in (options.rotor_radius, 0.55 * options.rotor_radius)]) \
        if options.rotor_radius > 0 and len(rotor_centres(parts)) else np.zeros((0, 3))
    try:
        _render(options, scene, stream, node, stop, snapshot, renderer, mesh, rotors)
    finally:
        renderer.close()


def _render(options, scene, stream, node, stop, snapshot, renderer, mesh, rotors):
    gear_z = float(mesh[:, 2].min()) if len(mesh) else -0.3     # the landing gear's feet, in base_link
    node.stats["gpu"] = renderer.gpu
    node.get_logger().info(f"renderer on {renderer.gpu}: {len(mesh) // 3} model triangles, "
                           f"{options.width}x{options.height} at {options.fps} fps")
    style = dict(STYLE, edl_strength=options.edl_strength)
    top, right, bottom, left = options.safe_area                   # fractions of the frame
    insets = (round(top * options.height), round(right * options.width),
              round(bottom * options.height), round(left * options.width))
    proj = perspective(options.fovy, options.width / options.height, 0.3, 3000.0)
    focal = options.height / 2 / math.tan(math.radians(options.fovy) / 2)
    camera = ChaseCamera(options.fovy, options.chase_distance, options.chase_max_distance)
    avia_mount, e1r_mount = options.frames[options.avia_frame], options.frames[options.e1r_frame]
    avia_lines, e1r_lines, ring = cone_lines(AVIA_FOV), pyramid_lines(E1R_FOV), ring_lines(1.0)
    span = float(np.ptp(mesh[:, :2], axis=0).max()) if len(mesh) else 1.0       # the airframe across, m
    zrange, zrange_time, hud_time, probe_time = (-1.0, 5.0), 0.0, 0.0, 0.0
    trail_version = scan_version = e1r_version = -1
    ground, ground_source, enclosed, agl, votes = None, "", False, 0.0, 0
    period, last, frame_times = 1.0 / options.fps, time.monotonic(), collections.deque(maxlen=60)
    rendering = False
    while not stop.is_set():
        start = time.monotonic()
        if not (snapshot is not None or options.always_on or (stream is not None and stream.active)):
            if rendering:
                renderer.clear_map()
                renderer.set_scan(None)
                renderer.set_e1r(None)
                renderer.reset_readback()
                rendering = False
            time.sleep(0.1)
            continue
        rendering = True
        with scene.lock:
            if scene.map_clear:
                renderer.clear_map()
                scene.map_clear = False
            chunks = list(scene.map_queue)
            scene.map_queue.clear()
            scan = scene.scan if scene.scan_version != scan_version else False
            e1r = scene.e1r if scene.e1r_version != e1r_version else False
            scan_version, e1r_version = scene.scan_version, scene.e1r_version
            latest_e1r, latest_scan = scene.e1r, scene.scan
            trail = scene.trail.view().copy() if scene.trail.version != trail_version else None
            trail_version = scene.trail.version
            sample, speed, start_z = scene.sample, scene.speed, scene.start_z
            avia_hz, e1r_hz = rate(scene.scan_times), rate(scene.e1r_times)
            leaps = sum(1 for j in scene.jumps if time.monotonic() - j < 10.0)
        for chunk in chunks:
            renderer.append_map(chunk)
        if scan is not False:
            renderer.set_scan(scan)
        if e1r is not False:
            renderer.set_e1r(e1r)
        if trail is not None:
            renderer.set_trail(trail)
        now = time.monotonic()
        if now - zrange_time > 2.0 and len(sample):
            centre = scene.poses.newest()
            zrange = robust_range(local_heights(sample, centre[1] if centre is not None else (0.0, 0.0)))
            zrange_time = now
        pose = scene.poses.at(scene.poses.render_time(now))
        model, overlay = None, np.zeros((0, 6), np.float32)
        eye, target = np.array([-options.chase_distance, 0.0, 3.0]), np.zeros(3)
        cut_target, clip_z = target, 1e9
        if pose is not None:
            position, q = pose
            model = np.eye(4)
            model[:3, :3] = quat_matrix(*q)
            model[:3, 3] = position
            if now - probe_time > 0.2:
                g = nadir_ground(latest_e1r, position[0], position[1], position[2], min(0.5 + 0.25 * agl, 6.0))
                ground_source = "E1R" if g is not None else ground_source or "takeoff"
                if g is None:
                    g = ground if ground is not None else (start_z if start_z is not None else position[2]) + gear_z
                ground = g if ground is None else ground + (1 - math.exp(-(now - probe_time) / 0.6)) * (g - ground)
                known, covered = scene.occupancy.enclosure(position[0], position[1], position[2], 5.0)
                inside = (covered >= 12 and covered >= 0.5 * known) or scan_enclosed(latest_scan, position)
                votes = min(votes + 1, 5) if inside else max(votes - 1, 0)    # ~0.6 s in, ~1 s out
                enclosed = votes >= 3 if not enclosed else votes > 0
                probe_time = now
            agl = max(0.0, float(position[2]) - ground)
            eye, target = camera.update(position, yaw_of(q), agl, now - last)
            cut_target = position
            clip_z = float(position[2]) + options.cutaway_above if enclosed else 1e9
            colors = [style["avia_fov_color"], style["e1r_fov_color"], style["plumb_color"], style["plumb_color"]]
            pieces = [place(avia_lines, model, avia_mount, min(max(0.18 * camera.length, 1.5), 6.0)),
                      place(e1r_lines, model, e1r_mount, min(max(agl + float(e1r_mount[2, 3]) - 0.05, 0.3), 80.0)),
                      np.array([position, (position[0], position[1], ground)]),
                      ring * max(0.45, 0.035 * camera.length) + np.array([position[0], position[1], ground + 0.02])]
            # Never smaller than model_min_px across (at most 3x true size): from a long boom the
            # airframe would be a speck; obstacles are far from the camera by then.
            apparent = span * focal / max(float(np.linalg.norm(position - eye)), 0.1)
            if options.model_min_px > 0 and apparent < options.model_min_px:
                model = model.copy()
                model[:3, :3] *= min(options.model_min_px / apparent, 3.0)
            if len(rotors):
                pieces.append(rotors @ model[:3, :3].T + model[:3, 3])
                colors.append(style["rotor_color"])
            overlay = np.concatenate([np.hstack([pc, np.tile(c, (len(pc), 1))]) for pc, c in zip(pieces, colors)])
        last = now
        renderer.set_overlay(overlay.astype(np.float32))
        if now - hud_time > 0.5:
            alt = float(pose[0][2]) - start_z if pose is not None and start_z is not None else 0.0
            lines = ["LIDAR MAP  ·  FAST-LIO2",
                     f"AGL {agl:5.1f} m   ALT {alt:+6.1f} m   SPD {speed:4.1f} m/s",
                     f"MAP {renderer.map_count / 1e6:4.2f} M   AVIA {avia_hz:4.1f} Hz   E1R {e1r_hz:4.1f} Hz"]
            legend = [("AVIA", style["avia_color"]), ("E1R", style["e1r_color"]), ("TRAIL", style["trail_color"])]
            footer = time.strftime("%Y-%m-%d %H:%M:%S UTC", time.gmtime()) + ("    CUTAWAY" if enclosed else "")
            verdict = scene.health
            if pose is None or not node.last_odom or now - node.last_odom > 2.0:
                warning = "NO FAST-LIO POSE"
            elif verdict and verdict.get("state") == "diverged":
                if verdict.get("exhausted"):
                    warning = "FAST-LIO DIVERGED \u00b7 MAP FROZEN \u00b7 AUTO-RESTART EXHAUSTED: REPOSITION, RESTART uav-lio"
                elif verdict.get("restart_in_s") is not None:
                    warning = f"FAST-LIO DIVERGED \u00b7 MAP FROZEN \u00b7 RESTARTING IT IN {verdict['restart_in_s']:.0f} s"
                else:
                    warning = "FAST-LIO DIVERGED \u00b7 MAP FROZEN"
            elif leaps >= 3:
                warning = f"FAST-LIO DIVERGED ({leaps} pose leaps in 10 s): RESTART uav-lio"
            elif speed > options.diverged_speed:
                warning = f"FAST-LIO DIVERGED ({speed:,.0f} m/s): RESTART uav-lio"
            else:
                warning = ""
            renderer.set_hud(hud_image(options.width, options.height, lines, legend, footer, options.font,
                                       warning=warning, insets=insets))
            hud_time = now
            node.stats.update(agl=agl, ground=ground_source, boom=camera.length, enclosed=enclosed)
        distance = float(np.linalg.norm(np.asarray(cut_target) - eye))
        frame = {"vp": (proj @ look_at(eye, target)).astype(np.float32), "eye": [float(v) for v in eye],
                 "focal": focal, "zrange": zrange, "clip_z": clip_z,
                 "cut_target": [float(v) for v in cut_target], "cut_radius": style["cut_radius"],
                 "cut_end": max(0.0, 1.0 - style["cut_stop_m"] / distance) if distance > 1e-3 else 0.0,
                 "intensity_ref": style["intensity_ref"], "model": model}
        pixels = renderer.render(frame, style)
        if snapshot is not None:
            pixels = renderer.render(frame, style)               # the frame just drawn
            image = np.frombuffer(pixels, np.uint8).reshape(options.height, options.width, 4)[::-1]
            from PIL import Image
            Image.fromarray(image, "RGBA").convert("RGB").save(snapshot)
            node.get_logger().info(f"snapshot {snapshot}: {renderer.map_count} map points, agl {agl:.2f} m "
                                   f"({ground_source}), boom {camera.length:.1f} m, enclosed {enclosed}")
            stop.set()
            break
        if pixels is not None and stream is not None:
            stream.push(pixels)
        elapsed = time.monotonic() - start
        frame_times.append(time.monotonic())
        node.stats["render_ms"] = 0.9 * node.stats.get("render_ms", elapsed * 1e3) + 0.1 * elapsed * 1e3
        node.stats["map_points"] = renderer.map_count
        node.stats["fps"] = rate(frame_times)
        time.sleep(max(0.0, period - (time.monotonic() - start)))


def rpy_quaternion(roll, pitch, yaw):
    """(x, y, z, w) of R = Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def first_returns(points, sensor, inside, max_range, bin_deg=0.25, cap=24000, rng=None):
    """What a LiDAR at pose `sensor` (4x4) would see of `points`: those in its field of view
    (`inside(x, y, z)` in its frame) within max_range, the nearest per angular bin (a crude
    z-buffer, so surfaces hide what is behind them), at most cap of them."""
    local = (points[:, :3] - sensor[:3, 3]) @ sensor[:3, :3]
    r = np.linalg.norm(local, axis=1)
    keep = (r > 0.5) & (r < max_range) & inside(local[:, 0], local[:, 1], local[:, 2])
    local, r, idx = local[keep], r[keep], np.flatnonzero(keep)
    az = np.degrees(np.arctan2(local[:, 1], local[:, 0])) // bin_deg
    el = np.degrees(np.arcsin(np.clip(local[:, 2] / r, -1, 1))) // bin_deg
    order = np.lexsort((r, el, az))
    key = (az[order] * 100000 + el[order])
    first = order[np.concatenate([[True], key[1:] != key[:-1]])]
    first = idx[first]
    if len(first) > cap:
        first = (rng or np.random.default_rng(0)).choice(first, cap, replace=False)
    return points[first]


def demo_site(scene, agl, heading_deg, avia_mount, e1r_mount, seed=7):
    """A synthetic site for --demo (no ROS): rolling ground with a road, trees and a building,
    the aircraft agl above it at the end of a curving trail, and live scans cut from the
    site by each sensor's field of view. For tuning the look and checking the framing."""
    rng = np.random.default_rng(seed)

    def ground_z(x, y):
        return 0.5 * np.sin(x / 9.0) * np.cos(y / 13.0) + 0.03 * x

    n = 2_600_000
    gx, gy = rng.uniform(-70, 70, n), rng.uniform(-70, 70, n)
    gi = rng.uniform(6, 40, n)
    off_road = np.abs(gy - 0.3 * gx - 6)
    gi[off_road < 3.0] = 18 + 4 * rng.standard_normal(int((off_road < 3.0).sum()))
    gi[off_road < 0.12] = 170                                  # the centre line
    parts = [np.stack([gx, gy, ground_z(gx, gy), gi], 1)]
    for _ in range(60):                                        # trees: a trunk and a canopy
        cx, cy = rng.uniform(-70, 70, 2)
        if abs(cy - 0.3 * cx - 6) < 6 or (abs(cx - 28) < 12 and abs(cy + 22) < 10) or math.hypot(cx, cy) < 6:
            continue
        base, h, r = ground_z(cx, cy), rng.uniform(6, 15), rng.uniform(1.8, 4.2)
        m = int(2500 * r * r)
        th, ph = rng.uniform(0, 2 * math.pi, m), np.arccos(rng.uniform(-1, 1, m))
        rad = r * (0.75 + 0.25 * rng.random(m))
        parts.append(np.stack([cx + rad * np.sin(ph) * np.cos(th), cy + rad * np.sin(ph) * np.sin(th),
                               base + h - r + 0.8 * rad * np.cos(ph), rng.uniform(15, 45, m)], 1))
        k = 800
        t, z = rng.uniform(0, 2 * math.pi, k), rng.uniform(0, h - r, k)
        parts.append(np.stack([cx + 0.25 * np.cos(t), cy + 0.25 * np.sin(t), base + z, rng.uniform(8, 20, k)], 1))
    bx, by, bw, bd, bh, ba = 28.0, -22.0, 16.0, 10.0, 8.0, math.radians(20)   # a building
    faces = []
    for (x0, y0), du, dv in (((-bw / 2, -bd / 2), bw, 0), ((-bw / 2, bd / 2), bw, 0),
                             ((-bw / 2, -bd / 2), 0, bd), ((bw / 2, -bd / 2), 0, bd)):   # four walls
        m = int(30 * (du + dv) * bh)
        s, z = rng.random(m), rng.uniform(0, bh, m)
        faces.append(np.stack([x0 + s * du, y0 + s * dv, z], 1))
    m = int(30 * bw * bd)
    faces.append(np.stack([rng.uniform(-bw / 2, bw / 2, m), rng.uniform(-bd / 2, bd / 2, m), np.full(m, bh)], 1))
    local = np.concatenate(faces)
    c, s = math.cos(ba), math.sin(ba)
    wx, wy = bx + c * local[:, 0] - s * local[:, 1], by + s * local[:, 0] + c * local[:, 1]
    parts.append(np.stack([wx, wy, ground_z(bx, by) + local[:, 2], rng.uniform(30, 70, len(local))], 1))
    site = np.concatenate(parts).astype(np.float32)
    scene.add_map(site)
    # The flight: a curve ending at the origin, heading_deg, agl up, nose down a little (cruise).
    yaw = math.radians(heading_deg)
    a = np.linspace(-1.0, 0.0, 400)
    along, across = 70 * a, 18 * a * a
    px = along * math.cos(yaw) - across * math.sin(yaw)
    py = along * math.sin(yaw) + across * math.cos(yaw)
    pz = ground_z(px, py) * 0.3 + agl * (0.6 + 0.4 * (1 - a * a)) + ground_z(0.0, 0.0) * 0.7
    for p in np.stack([px, py, pz], 1):
        scene.trail.add(p)
    position = np.array([0.0, 0.0, ground_z(0.0, 0.0) + agl])
    q = rpy_quaternion(0.0, math.radians(6), yaw)
    now = time.monotonic()
    for i in range(5):
        scene.poses.add(100.0 + 0.1 * i, position, q, arrival=now - 0.5 + 0.1 * i)
    pose = np.eye(4)
    pose[:3, :3], pose[:3, 3] = quat_matrix(*q), position
    avia, e1r = pose @ avia_mount, pose @ e1r_mount
    ta, tb = (math.tan(math.radians(v) / 2) for v in AVIA_FOV)
    scene.scan = first_returns(site, avia, lambda x, y, z: (x > 0) & ((y / (ta * x)) ** 2 + (z / (tb * x)) ** 2 < 1),
                               200.0, rng=rng)
    ea, eb = (math.tan(math.radians(v) / 2) for v in E1R_FOV)
    scene.e1r = first_returns(site, e1r, lambda x, y, z: (x > 0) & (np.abs(y) < ea * x) & (np.abs(z) < eb * x),
                              60.0, rng=rng)
    scene.scan_version = scene.e1r_version = 1
    scene.scan_times.extend(now - 0.1 * np.arange(10)[::-1])
    scene.e1r_times.extend(now - 0.1 * np.arange(10)[::-1])
    scene.speed, scene.start_z = 8.0, float(pz[0]) - agl * 0.6 + 0.3
    return position


class Offline:
    """Stands in for the ROS node under --demo: the render loop's stats, pose freshness and log."""

    def __init__(self):
        self.stats = {}
        self.last_odom = time.monotonic() + 3600.0

    def get_logger(self):
        return self

    def info(self, text):
        print(text, flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--map-topic", default="/lio/map/updates")
    parser.add_argument("--scan-topic", default="/cloud_registered")
    parser.add_argument("--e1r-topic", default="/lio/registered/e1r")
    parser.add_argument("--odometry-topic", default="/Odometry", help="FAST-LIO's IMU pose")
    parser.add_argument("--health-topic", default="/lio/health", help="the lio watchdog's verdict on FAST-LIO")
    parser.add_argument("--map-topic-base", dest="map_topic_base", default="/lio/map",
                        help="lio_map's base topic (its /epoch says when to reload)")
    parser.add_argument("--description-topic", default="/robot_description",
                        help="the URDF robot_state_publisher serves: the model and every mount below")
    parser.add_argument("--description-timeout", type=float, default=120.0, help="s to wait for it at start")
    parser.add_argument("--urdf", required=True, help="the model for --demo (and if the description never comes)")
    parser.add_argument("--package-dir", action="append", default=[], help="name=path for package:// URIs")
    parser.add_argument("--imu-frame", default="avia_imu", help="FAST-LIO's IMU (body) frame")
    parser.add_argument("--avia-frame", default="avia_nominal_lidar_frame", help="the Avia's LiDAR frame (its FOV)")
    parser.add_argument("--e1r-frame", default="e1r_nominal_lidar_frame", help="the E1R's LiDAR frame (its FOV)")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--bitrate", type=int, default=4_000_000)
    parser.add_argument("--codec", default="h265", choices=("h264", "h265"))
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8555)
    parser.add_argument("--mount", default="/lidar")
    parser.add_argument("--advertised-uri", default="")
    parser.add_argument("--fovy", type=float, default=50.0, help="vertical field of view, deg")
    parser.add_argument("--chase-distance", type=float, default=7.0, help="shortest boom, m")
    parser.add_argument("--chase-max-distance", type=float, default=90.0, help="longest boom, m")
    parser.add_argument("--cutaway-above", type=float, default=1.0,
                        help="while enclosed, hide map points more than this above the aircraft, m")
    parser.add_argument("--edl-strength", type=float, default=STYLE["edl_strength"])
    parser.add_argument("--safe-area", type=float, nargs=4, default=[0.10, 0.08, 0.16, 0.08],
                        metavar=("TOP", "RIGHT", "BOTTOM", "LEFT"),
                        help="overlay-free margins, fractions of the frame: the ground station's own bars cover them")
    parser.add_argument("--rotor-radius", type=float, default=0.25,
                        help="rotor disc rings at the URDF's motors, m (decorative; 0 = none)")
    parser.add_argument("--model-min-px", type=float, default=60.0,
                        help="draw the airframe at least this wide on screen (at most 3x true size); 0 = true size")
    parser.add_argument("--diverged-speed", type=float, default=40.0,
                        help="FAST-LIO speeds above this (m/s; no X950 flies so fast) are flagged as divergence")
    parser.add_argument("--voxel", type=float, default=0.05)
    parser.add_argument("--promote", type=int, default=3,
                        help="show a map cell (0.5 x 0.5 x 0.25 m) from its n-th voxel on: hides stray returns; 1 = all")
    parser.add_argument("--map-capacity", type=int, default=4_000_000)
    parser.add_argument("--trail-step", type=float, default=0.1)
    parser.add_argument("--font", default="/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf")
    parser.add_argument("--always-on", action="store_true", help="render without a viewer (testing)")
    parser.add_argument("--snapshot", default="", help="render one frame to this PNG after --wait s, then exit")
    parser.add_argument("--wait", type=float, default=5.0)
    parser.add_argument("--demo", type=float, default=None, metavar="AGL",
                        help="no ROS: render a synthetic site with the aircraft AGL metres up to --snapshot")
    parser.add_argument("--demo-heading", type=float, default=30.0, help="deg, ENU")
    options, ros_args = parser.parse_known_args()
    options.description = None
    if options.demo is not None:
        # No ROS: the URDF file, which lacks the description's runtime Avia frames; the Avia's
        # own link stands in for them in the synthetic scene.
        frames = urdf_frames(options.urdf)
        for name, fallbacks in ((options.avia_frame, ("avia_link",)), (options.imu_frame, (options.avia_frame, "avia_link"))):
            frames.setdefault(name, next(frames[f] for f in fallbacks if f in frames))
        options.frames = frames
        scene = Scene(options.voxel, options.map_capacity, options.trail_step, options.promote)
        demo_site(scene, options.demo, options.demo_heading, frames[options.avia_frame], frames[options.e1r_frame])
        options.always_on = True
        render_loop(options, scene, None, Offline(), threading.Event(), options.snapshot or "demo.png")
        return
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    rclpy.init(args=ros_args)
    scene = Scene(options.voxel, options.map_capacity, options.trail_step, options.promote)
    if options.snapshot:
        options.always_on = True
    stream = None if options.snapshot else RtspStream(options.bind, options.port, options.mount, options.width,
                                                      options.height, options.fps, options.bitrate, options.codec)
    node = create_node(options, scene, stream)
    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())        # systemd's stop: leave the loop, clean up
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True, name="lidar-view-ros")
    spin.start()
    try:
        if not node.description_ready.wait(options.description_timeout):
            raise SystemExit(f"no {options.description_topic} with {options.imu_frame}, {options.avia_frame} and "
                             f"{options.e1r_frame} in {options.description_timeout:.0f} s (is uav-description running?)")
        options.description, options.frames = node.description, node.frames
        if options.snapshot:
            stop.wait(options.wait)
        render_loop(options, scene, stream, node, stop, options.snapshot or None)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        if stream is not None:
            stream.close()
        executor.shutdown(timeout_sec=2.0)
        spin.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
