#!/bin/bash
# 3DGUT (MCMC) on a COLMAP-format export inside the 3dgrut container; usage: run_3dgut.sh <dataset dir under /srv/flights/nurec> <experiment>
set -eo pipefail
DS=$1; EXP=$2; shift 2
docker run --rm --gpus all --ipc=host --shm-size=16g -v /srv/flights/nurec:/data -w /workspace 3dgrut:cuda12 \
  bash -c "export PATH=/workspace/.venv/bin:\$PATH; exec python train.py --config-name apps/colmap_3dgut_mcmc.yaml \
    path=/data/$DS out_dir=/data/runs experiment_name=$EXP export_usd.enabled=true export_usd.format=nurec export_ply.enabled=true $*"
