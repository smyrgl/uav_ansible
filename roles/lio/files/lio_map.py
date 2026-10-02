#!/usr/bin/env python3
"""A voxel map of FAST-LIO's registered scans, for Foxglove and for saving.

FAST-LIO's own /Laser_map cannot serve: its 1 Hz timer appends only the
current scan to an unfiltered, ever-growing cloud and republishes all of it
every second, and its downsampled ikd-tree map is compiled out (if(0)). This
node keeps one point per voxel from every registered scan (/cloud_registered:
undistorted, in FAST-LIO's camera_init frame, which the lio bridge anchors
under odom), at two resolutions:

- fine (5 cm): only the newly filled voxels go out, every second, on
  /lio/map/updates, so the bandwidth follows new surface, not map size; a
  viewer accumulates them (Foxglove: a long decay time). When a subscriber
  appears, the whole fine map goes out once first, so a viewer that connects
  late gets the detail built before it. This is the map that /lio/map/save
  writes.
- overview (20 cm): the whole map, latched on /lio/map every few seconds, so a
  viewer that connects late still sees everything at once.

Services: /lio/map/save (std_srvs/Trigger) writes the fine map as a binary PCD
(x, y, z, intensity) into the map directory; /lio/map/reset clears both;
/lio/map/resend sends the whole fine map on /lio/map/updates again (a second
Foxglove client: the bridge shares one ROS subscription between clients, so
only the first one is noticed). A
gap in the scans longer than reset_gap_s (FAST-LIO restarted, new origin)
clears them too.
"""
import argparse
import os
import time
from datetime import datetime, timezone

import numpy as np

OFFSET = 1 << 20          # 21 bits per axis: +-1,048,576 voxels (+-210 km at 0.2 m)


def voxel_keys(xyz, voxel):
    """One int64 per point: its voxel's (ix, iy, iz), packed 21 bits each."""
    idx = np.floor(xyz / voxel).astype(np.int64) + OFFSET
    if idx.size and (idx.min() < 0 or idx.max() >= 1 << 21):
        raise ValueError("point outside the map's index range")
    return (idx[:, 0] << 42) | (idx[:, 1] << 21) | idx[:, 2]


class VoxelMap:
    """One point (the first seen) per voxel; x, y, z, intensity as float32."""

    def __init__(self, voxel, max_points):
        self.voxel = float(voxel)
        self.max_points = int(max_points)
        self.reset()

    def reset(self):
        self.keys = set()
        self.chunks = []
        self.size = 0
        self.full = False

    def add(self, points):
        """points: (n, 4) float32 x, y, z, intensity. Returns the new points (m, 4)."""
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
        self.keys.update(keys[fresh].tolist())
        new = points[first[fresh]].astype(np.float32)
        self.chunks.append(new)
        self.size += len(fresh)
        return new

    def points(self):
        if len(self.chunks) > 1:
            self.chunks = [np.concatenate(self.chunks)]
        return self.chunks[0] if self.chunks else np.zeros((0, 4), np.float32)


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
    from std_srvs.srv import Trigger

    class MapNode(Node):
        def __init__(self):
            super().__init__("lio_map")
            self.map = VoxelMap(options.voxel, options.max_points)
            self.overview = VoxelMap(options.overview_voxel, options.overview_max_points)
            self.frame, self.last_scan, self.dirty, self.published = None, None, False, 0
            self.pending, self.updates_sent = [], 0
            self.subscribers, self.resend = 0, False
            latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.pub = self.create_publisher(PointCloud2, options.map_topic, latched)
            self.updates_pub = self.create_publisher(PointCloud2, options.map_topic + "/updates", 10)
            self.create_subscription(PointCloud2, options.scan_topic, self._scan, 20)
            self.create_service(Trigger, options.map_topic + "/save", self._save)
            self.create_service(Trigger, options.map_topic + "/reset", self._reset)
            self.create_service(Trigger, options.map_topic + "/resend", self._resend)
            self.diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
            self._types = (PointCloud2, PointField, DiagnosticArray, DiagnosticStatus, KeyValue)
            self.create_timer(options.publish_period, self._publish)
            self.create_timer(options.update_period, self._publish_updates)
            self.create_timer(1.0, self._diagnose)
            self.get_logger().info(f"lio map: {options.scan_topic} -> {options.map_topic}/updates ({options.voxel} m, new voxels "
                                   f"every {options.update_period} s) and {options.map_topic} ({options.overview_voxel} m, "
                                   f"latched, every {options.publish_period} s)")

        def _scan(self, message):
            now = time.monotonic()
            if self.last_scan is not None and now - self.last_scan > options.reset_gap_s and self.map.size:
                self.get_logger().warning(f"no scan for {now - self.last_scan:.0f} s: new map (FAST-LIO restarted?)")
                self._clear()
            self.last_scan, self.frame = now, message.header.frame_id
            try:
                scan = xyzi(message)
                new = self.map.add(scan)
                if len(new):
                    self.pending.append(new)
                if len(self.overview.add(scan)):
                    self.dirty = True
            except (KeyError, ValueError) as exc:
                self.get_logger().warning(f"scan skipped: {exc}", throttle_duration_sec=10)

        def _clear(self):
            self.map.reset()
            self.overview.reset()
            self.pending, self.dirty = [], True

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
            values = {"points": self.map.size, "voxel_m": self.map.voxel, "max_points": self.map.max_points,
                      "overview_points": self.overview.size, "overview_voxel_m": self.overview.voxel,
                      "frame": self.frame, "overview_publishes": self.published, "updates_published": self.updates_sent}
            out = DiagnosticArray_()
            out.header.stamp = self.get_clock().now().to_msg()
            out.status = [DiagnosticStatus_(level=level, name="lio/map", hardware_id="FAST-LIO voxel map",
                                            message=text, values=[KeyValue_(key=k, value=str(v)) for k, v in values.items()])]
            self.diag_pub.publish(out)

    return MapNode()


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--scan-topic", default="/cloud_registered")
    parser.add_argument("--map-topic", default="/lio/map")
    parser.add_argument("--voxel", type=float, default=0.05, help="fine map voxel (updates, save), m")
    parser.add_argument("--max-points", type=int, default=4_000_000, help="fine map cap")
    parser.add_argument("--update-period", type=float, default=1.0, help="s between /updates messages")
    parser.add_argument("--overview-voxel", type=float, default=0.2, help="latched overview voxel, m")
    parser.add_argument("--overview-max-points", type=int, default=2_000_000)
    parser.add_argument("--publish-period", type=float, default=10.0, help="s between overview messages")
    parser.add_argument("--reset-gap-s", type=float, default=10.0)
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
