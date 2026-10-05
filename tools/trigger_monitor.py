#!/usr/bin/env python3
"""Read the pps-trigger MCU over USB CDC: verify checksums, print $HELLO/$PPS/$STAT/$EVT
lines and a per-second summary of the $TRG pulse offsets. Optionally switch the
self-test pulse (GPIO5) on or off first."""
import argparse
import glob
import statistics
import sys
import time

import serial


def checksum_ok(line):
    if not line.startswith("$") or "*" not in line:
        return False
    body, _, given = line[1:].partition("*")
    cs = 0
    for c in body.encode():
        cs ^= c
    return given[:2].upper() == "%02X" % cs


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", default="", help="serial device (default: first /dev/tty.usbmodem* or /dev/ttyACM*)")
    ap.add_argument("--test", choices=["on", "off"], help="send TEST 1 / TEST 0 first")
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--raw", action="store_true", help="print every $TRG line too")
    args = ap.parse_args()
    port = args.port or next(iter(sorted(glob.glob("/dev/tty.usbmodem*") + glob.glob("/dev/ttyACM*"))), None)
    if not port:
        sys.exit("no serial device found")
    with serial.Serial(port, 115200, timeout=0.2) as ser:
        ser.reset_input_buffer()
        ser.write(b"STATUS\n")
        if args.test:
            ser.write(b"TEST 1\n" if args.test == "on" else b"TEST 0\n")
        offsets, bad, status, second = [], 0, "?", None
        t_end = time.time() + args.seconds
        while time.time() < t_end:
            line = ser.readline().decode("ascii", "replace").strip()
            if not line:
                continue
            if not checksum_ok(line):
                bad += 1
                print("BAD CHECKSUM", line)
                continue
            f = line[1:].partition("*")[0].split(",")
            if f[0] == "TRG":
                _boot, _seq, n, _k, off_ns, st = f[1:7]
                status = st
                if second is None:
                    second = n
                if n != second:
                    if offsets:
                        print("%s N=%s status=%s pulses=%d offset_us mean %+.1f sd %.1f min %+.1f max %+.1f" % (
                            time.strftime("%H:%M:%S"), second, status, len(offsets), statistics.fmean(offsets),
                            statistics.pstdev(offsets), min(offsets), max(offsets)))
                    offsets, second = [], n
                offsets.append(int(off_ns) / 1000.0)
                if args.raw:
                    print(line)
            else:
                print(line)
        print("bad checksums: %d" % bad)


if __name__ == "__main__":
    main()
