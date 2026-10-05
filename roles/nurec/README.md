# NuRec reconstruction on the replay host

Installs what `docs/nurec.md` describes: 3DGRUT at a pinned commit built as the
project's own Docker image (`3dgrut:cuda12`, CUDA 12.8 for the RTX 5090), COLMAP from
apt for the mono-workflow pose refinement, the repo's reconstruction tools under
`/usr/local/lib/uav/nurec-tools/`, and one driver:

```sh
uav-reconstruct /srv/flights/flight_<UTC>            # frames, dataset, masks, 3DGUT, NuRec USDZ + PLY
uav-reconstruct /srv/flights/flight_<UTC> --sfm      # plus COLMAP SfM, metric alignment, a second run
```

Outputs under `/srv/flights/nurec/<bag>/` and `/srv/flights/nurec/runs/<bag>_3dgut_mcmc/`
(`export_last_nurec.usdz` for Isaac Sim, `export_last.ply`, `metrics.json` on the held-out
views). Steps are idempotent: a re-run skips what exists. The stereo-workflow pieces
(isaac_mapping_ros, pyCuSFM) are installed by hand and documented, not managed here; on
aerial data the IR pair's baseline is too short for them (docs/nurec.md).

```sh
ansible-playbook -i inventory/replay.yml site.yml --tags nurec
```
