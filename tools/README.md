# Tools (run on the Mac or the Jetson, outside Ansible)

## ulog2mcap.py — PX4 ulog to MCAP with UTC timestamps

Autonomy roadmap Stage 0b: the FC's ulog and the Jetson's bags must line up
to better than 2 ms so a replay can score the executor against what the pilot
flew. The ulog is in hrt microseconds since FC boot; the XRCE timesync
(`timesync_status.estimated_offset = px4 − companion`) converts it:
`UTC_us = hrt_us − estimated_offset`, interpolated between the 1 Hz samples.
The same log carries two independent references, reported as residuals of
that conversion:

| Reference | Meaning | Expected |
|---|---|---|
| `pps_capture.rtc_timestamp` | the receiver's UTC second at the hrt time of each PPS edge (FMU_CAP1) | median under 2 ms (the gate); exit code 3 above it |
| `vehicle_gps_position.time_utc_usec` | the fix epoch's UTC against the sample's hrt (fix type ≥ 3) | tens of ms (receiver latency); catches a sign or whole-second error |

Output: one channel per ulog dataset, `/ulog/<name>[_<instance>]`, JSON
messages with every field plus `utc_us`, `log_time = publish_time = UTC ns`;
`/ulog/parameters` (one message) and `/ulog/logged_messages`; an MCAP
metadata record `ulog2mcap` with the conversion summary (offset source,
residual statistics, counts). JSON rather than CDR so Foxglove and a Python
scorer read it without px4_msgs type support; zstd when `zstandard` is
installed.

```sh
python3 -m venv ~/ulogenv && ~/ulogenv/bin/pip install pyulog mcap zstandard
~/ulogenv/bin/python tools/ulog2mcap.py flight.ulg -o flight_ulog.mcap --summary flight_ulog.json
```

`--offset timesync|pps|none|auto` picks the reference (auto: timesync, then
the PPS edges themselves, then none: hrt kept as log_time). `--topics a,b`
limits the datasets. Tests: `python3 -m unittest test_ulog2mcap` in `tools/`
(the end-to-end case runs when `ULOG_SAMPLE` points at a log and pyulog/mcap
are installed; pyulog's own `test/sample.ulg` works but has no timesync or
PPS data, so the UTC path is covered by the synthetic tests until a log from
this FC is converted).
