#!/usr/bin/env python3
"""A voxel map of FAST-LIO's registered scans, for Foxglove and for saving.

FAST-LIO's own /Laser_map cannot serve: its 1 Hz timer appends only the
current scan to an unfiltered, ever-growing cloud and republishes all of it
every second, and its downsampled ikd-tree map is compiled out (if(0)). This
node keeps one point per voxel from every registered scan (/cloud_registered:
undistorted, in FAST-LIO's camera_init frame, which the lio bridge anchors
under odom), and from any other cloud registered into that frame with
FAST-LIO's poses (the E1R: /lio/registered/e1r, from lio_register), at two
resolutions:

- fine (5 cm): only the newly filled voxels go out, every second, on
  /lio/map/updates, so the bandwidth follows new surface, not map size; a
  viewer accumulates them (Foxglove: a long decay time). When a subscriber
  appears, the whole fine map goes out once first, so a viewer that connects
  late gets the detail built before it. This is the map that /lio/map/save
  writes.
- overview (20 cm): the whole map, latched on /lio/map every few seconds, so a
  viewer that connects late still sees everything at once.

Stray returns are kept out of both: a point joins only once its cell (0.5 m
across, 0.25 m tall) holds three distinct fine voxels (CellPromoter). Surfaces
fill their cells within a scan or two; the Avia's untagged stray far returns, at
random ranges along real beams, never do.

Services: /lio/map/save (std_srvs/Trigger) writes the fine map as a binary PCD
(x, y, z, intensity) into the map directory; /lio/map/reset clears both;
/lio/map/resend sends the whole fine map on /lio/map/updates again (a second
Foxglove client: the bridge shares one ROS subscription between clients, so
only the first one is noticed). A
gap in the scans longer than reset_gap_s (FAST-LIO restarted, new origin)
clears them too.

The lio watchdog's verdict (/lio/health) gates it: when FAST-LIO is declared
diverged the map freezes (scans refused) and takes back every point that arrived
since shortly before the onset; when FAST-LIO starts over (a new epoch) it
starts a new map. Either way /lio/map/epoch is bumped, so a viewer that
accumulates /updates drops what it holds, and the whole map follows.
"""
import argparse
import itertools
import os
import time
from collections import deque
from datetime import datetime, timezone

import numpy as np

import lio_health

OFFSET = 1 << 20          # 21 bits per axis: +-1,048,576 voxels (+-210 km at 0.2 m)


def voxel_keys(xyz, voxel):
    """One int64 per point: its voxel's (ix, iy, iz), packed 21 bits each."""
    idx = np.floor(xyz / voxel).astype(np.int64) + OFFSET
    if idx.size and (idx.min() < 0 or idx.max() >= 1 << 21):
        raise ValueError("point outside the map's index range")
    return (idx[:, 0] << 42) | (idx[:, 1] << 21) | idx[:, 2]


class VoxelMap:
    """One point (the first seen) per voxel; x, y, z, intensity as float32. The points are
    kept in arrival order, with a mark (monotonic s, size before) at every addition of the
    last `horizon_s`, so what arrived after a moment can be taken back (rollback)."""

    def __init__(self, voxel, max_points, horizon_s=600.0):
        self.voxel = float(voxel)
        self.max_points = int(max_points)
        self.horizon_s = float(horizon_s)
        self.reset()

    def reset(self):
        self.keys = set()
        self.chunks = []
        self.size = 0
        self.full = False
        self.marks = deque()

    def add(self, points, now=None):
        """points: (n, 4) float32 x, y, z, intensity, arrived at monotonic `now`.
        Returns the new points (m, 4)."""
        none = np.zeros((0, 4), np.float32)
        if self.full or not len(points):
            return none
        finite = np.isfinite(points[:, :3]).all(axis=1)
        points = points[finite]
        keys = voxel_keys(points[:, :3].astype(np.float64), self.voxel)
        keys, first = np.unique(keys, return_index=True)
        fresh = [i for i, k in enumerate(keys.tolist()) if k not in self.keys]
        if not fresh:
            return none
        room = self.max_points - self.size
        if len(fresh) > room:
            fresh, self.full = fresh[:room], True
        if now is not None:
            self.marks.append((now, self.size))
            while len(self.marks) > 1 and now - self.marks[0][0] > self.horizon_s:
                self.marks.popleft()
        self.keys.update(keys[fresh].tolist())
        new = points[first[fresh]].astype(np.float32)
        self.chunks.append(new)
        self.size += len(fresh)
        return new

    def rollback(self, since):
        """Take back every point that arrived at or after monotonic `since` (within the
        horizon). Returns how many."""
        keep = next((size for t, size in self.marks if t >= since), None)
        if keep is None or keep >= self.size:
            return 0
        pts = self.points()
        dropped = pts[keep:]
        self.keys.difference_update(voxel_keys(dropped[:, :3].astype(np.float64), self.voxel).tolist())
        self.chunks = [pts[:keep].copy()] if keep else []
        self.size, self.full = keep, False
        while self.marks and self.marks[-1][0] >= since:
            self.marks.pop()
        return len(dropped)

    def points(self):
        if len(self.chunks) > 1:
            self.chunks = [np.concatenate(self.chunks)]
        return self.chunks[0] if self.chunks else np.zeros((0, 4), np.float32)


class CellPromoter:
    """Keeps stray returns out of the map. A new point joins it only once its cell (`cell` m
    across, `level` m tall) holds `need` distinct fine voxels; then the cell's held points
    join together, and its later points go straight through. A surface fills its cells
    within a scan or two. The Avia's stray returns never do: about 0.3 % of its points
    indoors (2026-10-02), at random ranges along real beams out to ~430 m, not flagged by
    the Livox tag's noise bits. Each lands in a voxel of its own and deduplicates against
    nothing; they had become 36 % of the 5 cm map and 93 % of the overview. Held cells are
    capped, the oldest given up first (strays never complete a cell). need=1 passes everything."""

    MERGE = 50_000                      # promoted cells gathered before a bulk merge into the sorted index

    def __init__(self, need=3, cell=0.5, level=0.25, voxel=0.05, max_held_cells=100_000):
        self.need, self.cell, self.level, self.voxel = int(need), float(cell), float(level), float(voxel)
        self.max_held_cells = int(max_held_cells)
        self.reset()

    def reset(self):
        self.held = {}                  # cell -> [(fine voxel key, 16 bytes of x, y, z, intensity)]
        self.held_points = 0
        self.dropped_points = 0         # held points given up to the cap
        self._main = np.zeros(0, np.int64)      # promoted cells, sorted (8 bytes a cell)
        self._recent = np.zeros(0, np.int64)    # promoted since the last merge, sorted

    @property
    def promoted_cells(self):
        return len(self._main) + len(self._recent)

    def cell_keys(self, xyz):
        """One int64 per point: its cell's (ix, iy, iz), packed 21 bits each."""
        idx = np.floor(xyz / np.array([self.cell, self.cell, self.level])).astype(np.int64) + OFFSET
        if idx.size and (idx.min() < 0 or idx.max() >= 1 << 21):
            raise ValueError("point outside the map's index range")
        return (idx[:, 0] << 42) | (idx[:, 1] << 21) | idx[:, 2]

    def _is_promoted(self, cells):
        mask = np.zeros(len(cells), bool)
        for known in (self._main, self._recent):
            if len(known):
                at = np.minimum(np.searchsorted(known, cells), len(known) - 1)
                mask |= known[at] == cells
        return mask

    def seed(self, points):
        """Count the cells of points already in the map as promoted (after a rollback)."""
        if self.need > 1 and len(points):
            self._main = np.union1d(self._main, np.unique(self.cell_keys(points[:, :3].astype(np.float64))))

    def _promote(self, cells):
        self._recent = np.union1d(self._recent, np.asarray(cells, np.int64))
        if len(self._recent) > self.MERGE:      # an insert copies the whole index: merge in bulk
            self._main, self._recent = np.union1d(self._main, self._recent), np.zeros(0, np.int64)

    def filter(self, points):
        """points (n, 4) float32 -> those the map may take now: in promoted cells, or completing one."""
        if self.need <= 1 or not len(points):
            return points
        points = points[np.isfinite(points[:, :3]).all(axis=1)]
        if not len(points):
            return points
        xyz = points[:, :3].astype(np.float64)
        cells = self.cell_keys(xyz)
        unique, inverse = np.unique(cells, return_inverse=True)
        promoted = self._is_promoted(unique)[inverse]
        out = [points[promoted]]
        if promoted.all():
            return out[0]
        waiting = ~promoted
        pts, wcells, voxels = points[waiting], cells[waiting], voxel_keys(xyz[waiting], self.voxel)
        order = np.lexsort((voxels, wcells))                        # by cell, then voxel
        pts, wcells, voxels = pts[order], wcells[order], voxels[order]
        first = np.r_[True, (wcells[1:] != wcells[:-1]) | (voxels[1:] != voxels[:-1])]
        pts, wcells, voxels = pts[first], wcells[first], voxels[first]      # one point per voxel
        starts = np.flatnonzero(np.r_[True, wcells[1:] != wcells[:-1]])
        newly = []
        for s, e, c in zip(starts.tolist(), np.r_[starts[1:], len(wcells)].tolist(), wcells[starts].tolist()):
            entry = self.held.get(c)
            if entry is None and e - s >= self.need:                # a surface, complete in one scan
                out.append(pts[s:e])
                newly.append(c)
                continue
            if entry is None:
                entry = self.held[c] = []
            seen = {k for k, _ in entry}
            for k, row in zip(voxels[s:e].tolist(), pts[s:e]):
                if k not in seen:
                    seen.add(k)
                    entry.append((k, row.tobytes()))
                    self.held_points += 1
            if len(entry) >= self.need:
                out.append(np.frombuffer(b"".join(b for _, b in entry), np.float32).reshape(-1, 4))
                self.held_points -= len(entry)
                del self.held[c]
                newly.append(c)
        if newly:
            self._promote(newly)
        excess = len(self.held) - self.max_held_cells
        if excess > 0:                                              # oldest first (insertion order)
            for c in list(itertools.islice(self.held, excess + self.max_held_cells // 10)):
                n = len(self.held.pop(c))
                self.held_points -= n
                self.dropped_points += n
        return np.concatenate(out)


def pcd_bytes(points):
    """A binary PCD (x, y, z, intensity float32) of an (n, 4) float32 array."""
    n = len(points)
    header = ("# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z intensity\n"
              "SIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n"
              f"WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA binary\n")
    return header.encode() + np.ascontiguousarray(points, dtype="<f4").tobytes()


def xyzi(message):
    """x, y, z, intensity of a PointCloud2 with float32 fields of those names."""
    offsets = {f.name: f.offset for f in message.fields}
    dtype = np.dtype({"names": ["x", "y", "z", "intensity"], "formats": ["<f4"] * 4,
                      "offsets": [offsets[k] for k in ("x", "y", "z", "intensity")],
                      "itemsize": message.point_step})
    raw = np.frombuffer(bytes(message.data), dtype=dtype, count=message.width * message.height)
    return np.stack([raw["x"], raw["y"], raw["z"], raw["intensity"]], axis=1)


def create_map_node(options):
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2, PointField
    from std_msgs.msg import String, UInt32
    from std_srvs.srv import Trigger

    class MapNode(Node):
        def __init__(self):
            super().__init__("lio_map")
            self.map = VoxelMap(options.voxel, options.max_points)
            self.overview = VoxelMap(options.overview_voxel, options.overview_max_points)
            self.promoter = CellPromoter(options.promote, options.promote_cell, options.promote_level, options.voxel,
                                         options.max_held_cells)
            self.frame, self.last_scan, self.dirty, self.published = None, None, False, 0
            self.pending, self.updates_sent = [], 0
            self.subscribers, self.resend = 0, False
            self.health = lio_health.HealthFollower()   # the watchdog's verdict on FAST-LIO
            self.refused, self.rolled_back, self.map_epoch = 0, 0, 0
            latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.pub = self.create_publisher(PointCloud2, options.map_topic, latched)
            # Bumped whenever points are taken back or the map starts over: a viewer that
            # accumulates /updates drops what it has (the whole map follows on /updates).
            self.epoch_pub = self.create_publisher(UInt32, options.map_topic + "/epoch", latched)
            self._UInt32 = UInt32
            self.epoch_pub.publish(UInt32(data=0))
            self.create_subscription(String, options.health_topic, self._health, latched)
            self.updates_pub = self.create_publisher(PointCloud2, options.map_topic + "/updates", 10)
            self.sources = dict.fromkeys(options.scan_topics, 0)      # fine voxels each input filled first
            for topic in options.scan_topics:
                self.create_subscription(PointCloud2, topic, lambda message, topic=topic: self._scan(message, topic), 20)
            self.create_service(Trigger, options.map_topic + "/save", self._save)
            self.create_service(Trigger, options.map_topic + "/reset", self._reset)
            self.create_service(Trigger, options.map_topic + "/resend", self._resend)
            self.diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
            self._types = (PointCloud2, PointField, DiagnosticArray, DiagnosticStatus, KeyValue)
            self.create_timer(options.publish_period, self._publish)
            self.create_timer(options.update_period, self._publish_updates)
            self.create_timer(1.0, self._diagnose)
            self.get_logger().info(f"lio map: {', '.join(options.scan_topics)} -> {options.map_topic}/updates ({options.voxel} m, new voxels "
                                   f"every {options.update_period} s) and {options.map_topic} ({options.overview_voxel} m, "
                                   f"latched, every {options.publish_period} s)")

        def _scan(self, message, topic):
            if self.health.diverged:            # FAST-LIO's poses are garbage: map frozen
                self.refused += 1
                return
            now = time.monotonic()
            if self.last_scan is not None and now - self.last_scan > options.reset_gap_s and self.map.size:
                self.get_logger().warning(f"no scan for {now - self.last_scan:.0f} s: new map (FAST-LIO restarted?)")
                self._clear()
            if self.map.size and message.header.frame_id != self.frame:
                self.get_logger().warning(f"{topic} is in {message.header.frame_id}, the map in {self.frame}: skipped",
                                          throttle_duration_sec=10)
                return
            self.last_scan, self.frame = now, message.header.frame_id
            try:
                shown = self.promoter.filter(xyzi(message))     # stray returns never complete a cell
                new = self.map.add(shown, now)
                if len(new):
                    self.pending.append(new)
                    self.sources[topic] += len(new)
                # A coarse voxel seen for the first time holds a fine voxel seen for the first time (the
                # voxels nest), so the overview needs only the fine map's new points, until that is full.
                if len(self.overview.add(shown if self.map.full else new)):
                    self.dirty = True
            except (KeyError, ValueError) as exc:
                self.get_logger().warning(f"scan skipped: {exc}", throttle_duration_sec=10)

        def _clear(self):
            self.map.reset()
            self.overview.reset()
            self.promoter.reset()
            self.pending, self.dirty = [], True
            self.sources = dict.fromkeys(self.sources, 0)
            self._bump_epoch()

        def _bump_epoch(self):
            self.map_epoch += 1
            self.epoch_pub.publish(self._UInt32(data=self.map_epoch))

        def _health(self, message):
            verdict = lio_health.decode(message.data)
            if verdict is None:
                return
            event = self.health.update(verdict)
            if event == "restarted":
                self.get_logger().warning(f"FAST-LIO started over (epoch {verdict['epoch']}): new map")
                self._clear()
            elif event == "diverged":
                onset = verdict.get("onset_mono")
                taken = self._rollback(onset) if onset is not None else 0
                self.get_logger().error(f"FAST-LIO diverged ({verdict['reason']}): map frozen at {self.map.size} points, "
                                        f"{taken} that arrived since the onset taken back")

        def _rollback(self, since):
            """Take back what arrived since `since` (monotonic s); rebuild what derives from it."""
            taken = self.map.rollback(since)
            if taken:
                kept = self.map.points()
                self.overview.reset()
                self.overview.add(kept)
                self.promoter.reset()
                self.promoter.seed(kept)
                self.pending, self.dirty, self.resend = [], True, True
                self.rolled_back += taken
                self._bump_epoch()
            return taken

        def _cloud(self, pts):
            PointCloud2_, PointField_ = self._types[:2]
            msg = PointCloud2_()
            msg.header.frame_id = self.frame
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.height, msg.width = 1, len(pts)
            msg.fields = [PointField_(name=n, offset=4 * i, datatype=PointField_.FLOAT32, count=1)
                          for i, n in enumerate(("x", "y", "z", "intensity"))]
            msg.is_bigendian, msg.point_step, msg.row_step, msg.is_dense = False, 16, 16 * len(pts), True
            msg.data = np.ascontiguousarray(pts, dtype=np.float32).tobytes()
            return msg

        def _publish_updates(self):
            count = self.updates_pub.get_subscription_count()
            whole = self.resend or count > self.subscribers
            self.subscribers, self.resend = count, False
            if self.frame is None or not count:
                return
            if whole:
                pts, self.pending = self.map.points(), []     # the whole fine map for a new viewer
            elif self.pending:
                pts, self.pending = np.concatenate(self.pending), []
            else:
                return
            if len(pts):
                self.updates_pub.publish(self._cloud(pts))
                self.updates_sent += 1

        def _resend(self, _request, response):
            self.resend = True
            response.success, response.message = True, f"sending {self.map.size} points on the next update"
            return response

        def _publish(self):
            if not self.dirty or self.frame is None:
                return
            self.pub.publish(self._cloud(self.overview.points()))
            self.dirty, self.published = False, self.published + 1

        def _save(self, _request, response):
            pts = self.map.points()
            if not len(pts):
                response.success, response.message = False, "map is empty"
                return response
            os.makedirs(options.map_dir, exist_ok=True)
            path = os.path.join(options.map_dir, datetime.now(timezone.utc).strftime("lio_map_%Y%m%d_%H%M%SZ.pcd"))
            with open(path, "wb") as f:
                f.write(pcd_bytes(pts))
            response.success, response.message = True, f"{len(pts)} points, frame {self.frame}: {path}"
            self.get_logger().info(response.message)
            return response

        def _reset(self, _request, response):
            self._clear()
            response.success, response.message = True, "map cleared"
            return response

        def _diagnose(self):
            _, _, DiagnosticArray_, DiagnosticStatus_, KeyValue_ = self._types
            full = self.map.full or self.overview.full
            level = DiagnosticStatus_.WARN if full else DiagnosticStatus_.OK
            text = ("Map full (no longer growing)" if full
                    else f"{self.map.size} points at {self.map.voxel} m, {self.overview.size} at {self.overview.voxel} m")
            if self.health.diverged:
                level = DiagnosticStatus_.ERROR
                text = f"FAST-LIO diverged: map frozen at {self.map.size} points ({self.rolled_back} taken back)"
            values = {"points": self.map.size, "voxel_m": self.map.voxel, "max_points": self.map.max_points,
                      "overview_points": self.overview.size, "overview_voxel_m": self.overview.voxel,
                      "frame": self.frame, "overview_publishes": self.published, "updates_published": self.updates_sent,
                      **{f"points_from {topic}": n for topic, n in self.sources.items()},
                      "promote_after_voxels": self.promoter.need, "promoted_cells": self.promoter.promoted_cells,
                      "held_cells": len(self.promoter.held), "held_points": self.promoter.held_points,
                      "held_points_dropped": self.promoter.dropped_points,
                      "fastlio": self.health.verdict["state"] if self.health.verdict else "no verdict",
                      "scans_refused": self.refused, "points_taken_back": self.rolled_back, "map_epoch": self.map_epoch}
            out = DiagnosticArray_()
            out.header.stamp = self.get_clock().now().to_msg()
            out.status = [DiagnosticStatus_(level=level, name="lio/map", hardware_id="FAST-LIO voxel map",
                                            message=text, values=[KeyValue_(key=k, value=str(v)) for k, v in values.items()])]
            self.diag_pub.publish(out)

    return MapNode()


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--scan-topics", nargs="+", default=["/cloud_registered"],
                        help="registered clouds, all in FAST-LIO's frame")
    parser.add_argument("--map-topic", default="/lio/map")
    parser.add_argument("--voxel", type=float, default=0.05, help="fine map voxel (updates, save), m")
    parser.add_argument("--max-points", type=int, default=4_000_000, help="fine map cap")
    parser.add_argument("--update-period", type=float, default=1.0, help="s between /updates messages")
    parser.add_argument("--overview-voxel", type=float, default=0.2, help="latched overview voxel, m")
    parser.add_argument("--overview-max-points", type=int, default=2_000_000)
    parser.add_argument("--publish-period", type=float, default=10.0, help="s between overview messages")
    parser.add_argument("--reset-gap-s", type=float, default=10.0)
    parser.add_argument("--promote", type=int, default=3,
                        help="map a cell's points only once it holds this many fine voxels (1 = every point)")
    parser.add_argument("--promote-cell", type=float, default=0.5, help="promotion cell across, m")
    parser.add_argument("--promote-level", type=float, default=0.25, help="promotion cell height, m")
    parser.add_argument("--max-held-cells", type=int, default=100_000, help="cells held back at most")
    parser.add_argument("--health-topic", default="/lio/health", help="the lio watchdog's verdict on FAST-LIO")
    parser.add_argument("--map-dir", default="/data/maps")
    options, ros_args = parser.parse_known_args()
    import rclpy
    node = None
    rclpy.init(args=ros_args)
    try:
        node = create_map_node(options)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
