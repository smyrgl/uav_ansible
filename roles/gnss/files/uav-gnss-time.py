#!/usr/bin/env python3
"""uav_ansible GNSS time feed: SBF ReceiverTime -> chrony SOCK refclock.

Reads the receiver's SBF stream from the GNSS fan-out (uav-gnss-broker) and,
for every ReceiverTime block (5914, once a second) that says the receiver
clock is synchronised to GNSS time, sends chrony one sample: "when this block
arrived the system clock read t, and true UTC was T". chrony's `refclock SOCK
... refid GPS` takes it; the GPIO PPS (`refclock PPS ... lock GPS`) uses it to
number its pulses. This source says which second it is, the PPS says exactly
when it began.

A sample is sent only when (SBF Reference Guide 4.2.12, 2.3):
  * the block's CRC checks out;
  * SyncLevel has WNSET, TOWSET and FINETIME, the manual's "full
    synchronization": receiver time within 0.5 ms of GPS time, which takes a
    first fix. FINETIME then latches until the receiver resets, so through an
    outage this keeps reporting the receiver's free-running clock (a TCXO:
    milliseconds of drift per hour at worst). That is still good enough to
    number seconds; precision is the PPS's job, and the receiver stops the PPS
    once it has no PVT;
  * DeltaLS (GPS-UTC, leap seconds) is known: no guessing 18;
  * the block's own UTC date and time agree, to the second, with
        UTC = GPS epoch (1980-01-06, Unix 315964800) + WNc weeks + TOW - DeltaLS

Samples are stamped when the block arrives here, so they run late by the
receiver's output latency (epoch -> SBF -> USB -> fan-out -> here): a bias of
tens of ms that the refclock's `offset` calibrates out (group_vars
gnss_time_offset). The PPS lock accepts it either way: it only needs this
source within ~0.2 s to pick the right second.

Config via environment: HOST, PORT (the fan-out), CHRONY_SOCK (chronyd creates
it; root-owned 0755, which is why this runs as root, with no capabilities).
"""
import binascii
import datetime
import os
import socket
import struct
import sys
import time

HOST = os.environ.get("HOST", "127.0.0.1")
PORT = int(os.environ.get("PORT", "28785"))
CHRONY_SOCK = os.environ.get("CHRONY_SOCK", "/run/chrony.gnss.sock")

RECEIVER_TIME = 5914
GPS_EPOCH_UNIX = 315964800
FULL_SYNC = 0b111  # SyncLevel bits 0-2: WNSET, TOWSET, FINETIME
# chrony's struct sock_sample (refclock_sock.c): struct timeval tv; double
# offset; int pulse; int leap; int _pad; int magic. Native layout: 40 bytes on
# a 64-bit ABI, and chrony drops a datagram of any other length.
SAMPLE = struct.Struct("@lldiiii")
SOCK_MAGIC = 0x534F434B

_last_note = None


def note(msg):
    """Log state changes, not every second of the same state."""
    global _last_note
    if msg != _last_note:
        print("[gnss-time]", msg, file=sys.stderr, flush=True)
        _last_note = msg


def receiver_times(buf):
    """(complete CRC-valid ReceiverTime blocks in buf, unconsumed tail).

    Looks for this one block type only, so it never has to trust another
    block's length field: a sync word followed by anything but a plausible
    ReceiverTime header is skipped.
    """
    found, i = [], 0
    while True:
        i = buf.find(b"$@", i)
        if i < 0:
            return found, buf[-1:]  # a trailing "$" may be half a sync word
        if len(buf) < i + 8:
            return found, buf[i:]
        crc, bid, ln = struct.unpack_from("<HHH", buf, i + 2)
        if bid & 0x1FFF != RECEIVER_TIME or not 24 <= ln <= 64 or ln % 4:
            i += 2
            continue
        if len(buf) < i + ln:
            return found, buf[i:]
        blk = buf[i:i + ln]
        if binascii.crc_hqx(blk[4:], 0) == crc:  # CRC-CCITT over ID..end
            found.append(blk)
            i += ln
        else:
            i += 2


def utc_ms(blk):
    """(UTC of the block's epoch in Unix ms, None) or (None, why not)."""
    tow, wnc = struct.unpack_from("<IH", blk, 8)
    y, mo, d, h, mi, s, delta_ls = struct.unpack_from("<7b", blk, 14)
    sync = blk[21]
    if sync & FULL_SYNC != FULL_SYNC:
        return None, (f"receiver time not synchronised yet (SyncLevel 0x{sync:02x}; "
                      "FINETIME needs a first fix)")
    if tow == 0xFFFFFFFF or wnc == 0xFFFF:
        return None, "TOW/WNc not set"
    if delta_ls == -128:
        return None, "leap seconds (DeltaLS) not decoded yet"
    t = (GPS_EPOCH_UNIX + wnc * 604800 - delta_ls) * 1000 + tow
    try:  # 2-digit year; -128 is do-not-use; a leap second's :60 also lands here
        stated = datetime.datetime(2000 + y, mo, d, h, mi, s, tzinfo=datetime.timezone.utc)
    except ValueError:
        return None, f"UTC fields unusable ({y} {mo} {d} {h}:{mi}:{s})"
    if int(stated.timestamp()) != t // 1000:
        return None, (f"UTC fields ({stated:%Y-%m-%d %H:%M:%S}) disagree with "
                      f"WNc {wnc} TOW {tow} DeltaLS {delta_ls}")
    return t, None


def sample(rx_ns, t_ms):
    """chrony SOCK datagram: at system time rx_ns, true time was t_ms."""
    sec, ns = divmod(rx_ns, 1_000_000_000)
    usec = ns // 1000  # timeval resolution; the offset is relative to exactly this
    offset_ns = t_ms * 1_000_000 - (sec * 1_000_000_000 + usec * 1000)
    return SAMPLE.pack(sec, usec, offset_ns / 1e9, 0, 0, 0, SOCK_MAGIC)


def main():
    out = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    while True:
        try:
            src = socket.create_connection((HOST, PORT), timeout=5)
        except OSError as e:
            note(f"fan-out {HOST}:{PORT} unreachable: {e}")
            time.sleep(2)
            continue
        src.settimeout(10)
        buf = b""
        try:
            while True:
                data = src.recv(65536)
                rx_ns = time.clock_gettime_ns(time.CLOCK_REALTIME)
                if not data:
                    raise ConnectionError("fan-out closed the connection")
                found, buf = receiver_times(buf + data)
                for blk in found:
                    t_ms, why = utc_ms(blk)
                    if t_ms is None:
                        note(f"waiting: {why}")
                        continue
                    # sendto by path, not connect(): chronyd re-creates the
                    # socket on every restart, and a connected socket would
                    # keep pointing at the old, unlinked one.
                    try:
                        out.sendto(sample(rx_ns, t_ms), CHRONY_SOCK)
                    except OSError as e:
                        note(f"chrony is not listening on {CHRONY_SOCK}: {e}")
                        continue
                    note(f"feeding chrony ({CHRONY_SOCK}) once a second")
        except OSError as e:  # includes the recv timeout
            note(f"fan-out stream lost: {e}; reconnecting")
        finally:
            src.close()
        time.sleep(2)


if __name__ == "__main__":
    main()
