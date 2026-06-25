#!/usr/bin/env bash
# uav_ansible — GNSS serial broker  (PLACEHOLDER / STARTING POINT)
#
# Goal: one process owns the single Mosaic serial and provides:
#   * READ fan-out of the SBF stream to N TCP subscribers:
#       - GPSD_PORT   -> gpsd        (time SHM + the ONLY writer, for RTCM)
#       - DRIVER_PORT -> septentrio_gnss_driver (read-only, rich data)
#   * a single arbitrated WRITE channel back to the serial (RTCM from gpsd).
#
# Why this is non-trivial: a single serial byte stream cannot be cleanly forked
# once something also writes back — independent writers interleave and corrupt
# the command/correction stream. So: tee reads freely, but accept writes from
# exactly ONE source (gpsd). Candidates to implement this properly:
#   - RTKLIB `str2str` (serial <-> multiple TCP)
#   - `ser2net`
#   - a small custom multiplexer (Python/pyserial + asyncio TCP servers)
#
# The naive socat sketch below tees reads but does NOT yet arbitrate the write
# path — DO NOT ship as-is. Prototype and measure before enabling the unit.
set -euo pipefail

: "${MOSAIC_DEV:?}" "${MOSAIC_BAUD:?}" "${GPSD_PORT:?}" "${DRIVER_PORT:?}"

echo "uav-gnss-broker: PLACEHOLDER — implement read fan-out + single-writer arbitration." >&2
echo "  serial=${MOSAIC_DEV}@${MOSAIC_BAUD} gpsd=${GPSD_PORT} driver=${DRIVER_PORT}" >&2

# --- Sketch (read tee only; write path TODO) --------------------------------
# socat -d -d \
#   "GOPEN:${MOSAIC_DEV},b${MOSAIC_BAUD},raw,echo=0" \
#   "TCP-LISTEN:${GPSD_PORT},reuseaddr,fork" &
# ... plus a second listener on DRIVER_PORT and write arbitration. TODO.

exit 1
