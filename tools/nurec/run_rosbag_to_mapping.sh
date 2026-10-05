#!/bin/bash
# Stage 1 of NVIDIA's stereo NuRec workflow: the bag to cuSFM's input layout with isaac_mapping_ros
# (ros-jazzy-isaac-mapping-ros). Poses from PX4's ENU odometry (child frame base_link, the default the tool
# expects), camera extrinsics through /tf_static. Usage: run_rosbag_to_mapping.sh <bag dir or .mcap> <out dir> [config yaml]
set -eo pipefail
BAG=$1; OUT=$2; shift 2
CFG=$(dirname "$0")/d555_ir_stereo.yaml
if [ $# -gt 0 ] && [ "${1#--}" = "$1" ]; then CFG=$1; shift; fi      # optional config path, then tool options
[ -d "$BAG" ] && BAG=$(ls "$BAG"/*.mcap | head -1)
source /opt/ros/jazzy/setup.bash
mkdir -p "$OUT"
ros2 run isaac_mapping_ros rosbag_to_mapping_data \
  --sensor_data_bag_file "$BAG" --output_folder_path "$OUT" \
  --pose_topic_name /px4/odometry --camera_topic_config "$CFG" \
  --min_inter_frame_distance 0.5 --min_inter_frame_rotation_degrees 5 "$@"
