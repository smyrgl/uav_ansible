#!/bin/bash
# Stage 2 of NVIDIA's stereo NuRec workflow: cuSFM pose estimation on a rosbag_to_mapping_data output
# (sequential stereo: cuVSLAM provides the initial trajectory, then global bundle adjustment). Exports COLMAP
# binaries too, for 3DGRUT and for the metric alignment. Usage: run_cusfm.sh <mapping dir> <cusfm out dir>
set -eo pipefail
IN=$1; OUT=$2; shift 2
source "${PYCUSFM_VENV:-$HOME/src/pycusfm-venv}/bin/activate"
mkdir -p "$OUT"
cusfm_cli --input_dir "$IN" --cusfm_base_dir "$OUT" \
  --min_inter_frame_distance 0.06 --min_inter_frame_rotation_degrees=1.5 --export_binary_colmap_files "$@"
