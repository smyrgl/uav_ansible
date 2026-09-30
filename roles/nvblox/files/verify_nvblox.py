#!/usr/bin/env python3
"""Bench check for the nvblox reconstruction on jethawk.

Run on the Jetson with the bench ROS environment, e.g.

    export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
           CYCLONEDDS_URI=file:///etc/uav/ros/cyclonedds.xml ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
    source /opt/ros/jazzy/setup.bash
    python3 /usr/local/lib/uav/verify_nvblox.py --wait 40 --save-ply /var/lib/uav-ros/nvblox/bench.ply

Subscribing here is what makes nvblox publish its layer markers and ESDF (it
only serialises layers that have subscribers). The script asserts:

* TF: odom -> base_link -> {camera_depth_optical_frame, camera_color_optical_frame,
  avia_nominal_lidar_frame} resolves from /tf_static alone.
* The rectified colour adapter publishes rgb8 with a zero-distortion CameraInfo
  carrying identical stamps.
* nvblox publishes a coloured surface-voxel Marker (CUBE_LIST) and, in 3d mode,
  an ESDF point cloud, both in the global frame.
* The unit's journal (current boot) shows no LiDAR-intrinsics or encoding errors,
  and reports the integration rates nvblox prints.
* Optionally, save_ply writes a non-empty mesh file.

Prints a JSON summary; exits non-zero on the first failed assertion.
"""

import argparse
import json
import os
import re
import subprocess
import sys
import time


def journal(unit, lines=400):
    try:
        out = subprocess.run(
            ["journalctl", "-u", unit, "-b", "--no-pager", "-o", "cat", "-n", str(lines)],
            check=False, capture_output=True, text=True, timeout=20,
        )
        return out.stdout
    except (OSError, subprocess.SubprocessError) as error:
        return f"<journal unavailable: {error}>"


def latest_rates(text):
    """Return {name: hz} from the last 'Rates statistics' block nvblox printed."""
    blocks = text.split("Rates statistics:")
    if len(blocks) < 2:
        return {}
    rates = {}
    for line in blocks[-1].splitlines()[1:80]:
        match = re.match(r"\s*(ros/\S+)\s+(\d+)\s+([-+0-9.eE]+)", line)
        if match:
            rates[match.group(1)] = float(match.group(3))
        elif rates and not line.strip():
            break
    return rates


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--wait", type=float, default=30.0, help="seconds to collect messages")
    parser.add_argument("--global-frame", default="odom")
    parser.add_argument("--pose-frame", default="base_link")
    parser.add_argument("--voxel", type=float, default=0.05)
    parser.add_argument("--unit", default="uav-nvblox.service")
    parser.add_argument("--rgb-namespace", default="/d555/color/rect")
    parser.add_argument("--save-ply", default=None, help="path (writable by uav-ros) for a mesh PLY")
    parser.add_argument("--no-journal", action="store_true")
    options = parser.parse_args()

    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CameraInfo, Image, PointCloud2
    from tf2_msgs.msg import TFMessage
    from visualization_msgs.msg import Marker

    rclpy.init()
    node = rclpy.create_node("verify_nvblox_bench")
    edges = {}
    markers = {"color": [], "tsdf": []}
    esdf = []
    rgb = []
    infos = []

    def on_tf(message):
        for transform in message.transforms:
            edges[transform.child_frame_id] = transform.header.frame_id

    def marker_cb(kind):
        def callback(message):
            markers[kind].append((message.header.frame_id, len(message.points), len(message.colors),
                                  message.type, round(message.scale.x, 4)))
        return callback

    best_effort = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
    # Two static publishers latch on /tf_static (the description and the bench
    # odometry stand-in); a depth-1 reader keeps only one of them. tf2_ros uses 100.
    static_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(TFMessage, "/tf_static", on_tf, static_qos)
    node.create_subscription(Marker, "/nvblox_node/color_layer_marker", marker_cb("color"), 10)
    node.create_subscription(Marker, "/nvblox_node/tsdf_layer_marker", marker_cb("tsdf"), 10)
    node.create_subscription(PointCloud2, "/nvblox_node/static_esdf_pointcloud",
                             lambda m: esdf.append((m.header.frame_id, m.width * m.height,
                                                    [f.name for f in m.fields])), 10)
    node.create_subscription(Image, f"{options.rgb_namespace}/image",
                             lambda m: rgb.append((m.header.stamp.sec, m.header.stamp.nanosec, m.encoding,
                                                   m.width, m.height, m.header.frame_id)), best_effort)
    node.create_subscription(CameraInfo, f"{options.rgb_namespace}/camera_info",
                             lambda m: infos.append((m.header.stamp.sec, m.header.stamp.nanosec,
                                                     tuple(m.d), m.header.frame_id)), best_effort)

    end = time.monotonic() + options.wait
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.2)

    def chain(frame):
        path = [frame]
        while frame != options.global_frame:
            assert frame in edges, f"TF: no static parent for {frame}; have {sorted(edges)}"
            frame = edges[frame]
            assert frame not in path, "TF loop"
            path.append(frame)
        return path

    summary = {"wait_s": options.wait}
    summary["tf_chains"] = {
        f: chain(f) for f in ("camera_depth_optical_frame", "camera_color_optical_frame",
                              "avia_nominal_lidar_frame")
    }
    assert edges.get(options.pose_frame) == options.global_frame, \
        f"TF: expected static {options.global_frame} -> {options.pose_frame} (bench odometry stand-in)"

    assert rgb, "no rectified colour images"
    assert all(e == "rgb8" and f == "camera_color_optical_frame" for _, _, e, _, _, f in rgb), rgb[:3]
    stamps_rgb = {(s, n) for s, n, *_ in rgb}
    stamps_info = {(s, n) for s, n, *_ in infos}
    assert stamps_rgb & stamps_info, "rectified image and CameraInfo stamps never match"
    assert all(all(v == 0.0 for v in d) for _, _, d, _ in infos), "rectified CameraInfo must have zero distortion"
    summary["rgb"] = {"frames": len(rgb), "rate_hz": round((len(rgb) - 1) / options.wait, 2),
                      "size": f"{rgb[-1][3]}x{rgb[-1][4]}", "stamp_matches": len(stamps_rgb & stamps_info)}

    assert markers["color"], "no /nvblox_node/color_layer_marker received (is nvblox integrating?)"
    frame, points, colors, mtype, scale = markers["color"][-1]
    assert frame == options.global_frame, f"marker frame {frame}"
    assert mtype == Marker.CUBE_LIST and colors == points and points > 0, markers["color"][-1]
    assert abs(scale - (options.voxel - 1e-3)) < 1e-3, f"cube scale {scale} vs voxel {options.voxel}"
    summary["color_layer_marker"] = {"messages": len(markers["color"]), "surface_voxels": points}
    if markers["tsdf"]:
        summary["tsdf_layer_marker"] = {"messages": len(markers["tsdf"]), "voxels": markers["tsdf"][-1][1]}
    if esdf:
        frame, count, fields = esdf[-1]
        assert frame == options.global_frame, f"esdf frame {frame}"
        summary["static_esdf_pointcloud"] = {"messages": len(esdf), "points": count, "fields": fields}
    else:
        summary["static_esdf_pointcloud"] = "none received"

    if not options.no_journal:
        text = journal(options.unit)
        bad = [line for line in text.splitlines() if re.search(
            r"LiDAR intrinsics are inconsistent|Invalid (color|depth|mask) image encoding|"
            r"Failed to transform|Check failed|Aborted", line)]
        assert not bad, "nvblox journal errors:\n" + "\n".join(bad[:10])
        summary["nvblox_rates_hz"] = latest_rates(text)
        summary["journal_lines_checked"] = len(text.splitlines())

    if options.save_ply:
        from nvblox_msgs.srv import FilePath
        client = node.create_client(FilePath, "/nvblox_node/save_ply")
        assert client.wait_for_service(timeout_sec=10.0), "save_ply service not available"
        future = client.call_async(FilePath.Request(file_path=options.save_ply))
        rclpy.spin_until_future_complete(node, future, timeout_sec=60.0)
        assert future.done() and future.result() is not None and future.result().success, \
            f"save_ply failed: {future.result()}"
        deadline = time.monotonic() + 10.0
        size = 0
        while time.monotonic() < deadline:
            if os.path.exists(options.save_ply):
                size = os.path.getsize(options.save_ply)
                if size > 0:
                    break
            time.sleep(0.5)
        assert size > 0, f"{options.save_ply} is missing or empty"
        summary["ply"] = {"path": options.save_ply, "bytes": size}

    print(json.dumps(summary, indent=2), flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except AssertionError as error:
        print(f"FAIL: {error}", file=sys.stderr)
        sys.exit(1)
