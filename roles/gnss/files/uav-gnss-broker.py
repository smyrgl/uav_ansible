#!/usr/bin/env python3
"""uav_ansible GNSS fan-out: one reader on the receiver's data port, many
read-only listeners on one localhost TCP port.

A tty hands each byte to exactly one read(): two processes reading /dev/gnss
would each get a random, corrupt share of the SBF stream. So this process is
the port's only reader (it sets TIOCEXCL, so the kernel refuses any other
non-root open while it runs) and copies every byte to every client of PORT:
the time feed (uav-gnss-time), the ROS driver, ad-hoc diagnostics.

Read-only. Nothing a client sends reaches the receiver: it is read and
discarded, and logged once per client. The receiver's other USB port (USB1)
is its command port, owned by drone-link, which feeds it RTCM; a second writer
here could interleave with that mid-message.

Never blocks on a client. Everything is non-blocking in one event loop: a
client that stops reading gets a private queue, and once that queue passes
MAX_BACKLOG bytes it is disconnected (it reconnects and resynchronises on the
next SBF sync word). A blocking send would stall the serial read instead, the
receiver's USB buffer would overflow (RxError CONGESTION), and every other
client would starve with the slow one.

Config via environment: SERIAL_DEV, SERIAL_BAUD (ignored by USB CDC ports),
PORT, BIND (default 127.0.0.1), MAX_BACKLOG (default 1 MiB, a few seconds of
the 100 Hz stream).
Resilient: reopens the serial after an unplug or error, and keeps clients
connected across the gap.
"""
import fcntl
import os
import selectors
import socket
import sys
import termios
import time

import serial  # python3-serial (pyserial)

SERIAL_DEV = os.environ.get("SERIAL_DEV", "/dev/gnss")
SERIAL_BAUD = int(os.environ.get("SERIAL_BAUD", "115200"))
PORT = int(os.environ.get("PORT", "28785"))
BIND = os.environ.get("BIND", "127.0.0.1")
MAX_BACKLOG = int(os.environ.get("MAX_BACKLOG", str(1 << 20)))
REOPEN_S = 2.0
SILENT_S = 5.0  # the port streams at 100 Hz; this long without a byte is news

sel = selectors.DefaultSelector()
clients = {}  # socket -> Client


def log(*a):
    print("[gnss-fanout]", *a, file=sys.stderr, flush=True)


def once(msg, last):
    """Log msg unless it repeats `last`; return it as the new `last`."""
    if msg != last:
        log(msg)
    return msg


class Client:
    def __init__(self, sock, addr):
        self.sock = sock
        self.name = "%s:%d" % addr[:2]
        self.queue = bytearray()  # what the kernel's send buffer would not take
        self.discarded = 0
        self.dropped = False


def drop(c, why):
    # Idempotent: one select() pass can report a client twice over (a failed
    # send while fanning out serial data, then its own pending event).
    if c.dropped:
        return
    c.dropped = True
    try:
        sel.unregister(c.sock)
    except (KeyError, ValueError):
        pass
    clients.pop(c.sock, None)
    c.sock.close()
    log(f"client {c.name} dropped: {why} ({len(clients)} connected)")


def accept(lsock):
    try:
        sock, addr = lsock.accept()
    except BlockingIOError:
        return
    sock.setblocking(False)
    # The stream is many small SBF blocks; Nagle would hold them back waiting
    # for ACKs, and a client's timestamps would inherit that jitter.
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    c = Client(sock, addr)
    clients[sock] = c
    sel.register(sock, selectors.EVENT_READ, c)
    log(f"client {c.name} connected ({len(clients)} connected)")


def send(c, data):
    if not c.queue:
        try:
            n = c.sock.send(data)
        except BlockingIOError:
            n = 0
        except OSError as e:
            drop(c, e)
            return
        if n == len(data):
            return
        data = data[n:]
        sel.modify(c.sock, selectors.EVENT_READ | selectors.EVENT_WRITE, c)
    c.queue += data
    if len(c.queue) > MAX_BACKLOG:
        drop(c, f"not reading ({len(c.queue)} bytes queued)")


def flush(c):
    try:
        n = c.sock.send(c.queue)
    except BlockingIOError:
        return
    except OSError as e:
        drop(c, e)
        return
    del c.queue[:n]
    if not c.queue:
        sel.modify(c.sock, selectors.EVENT_READ, c)


def discard(c):
    try:
        data = c.sock.recv(4096)
    except BlockingIOError:
        return
    except OSError as e:
        drop(c, e)
        return
    if not data:
        drop(c, "disconnected")
        return
    if not c.discarded:
        log(f"client {c.name} wrote to the port; discarding: it is read-only")
    c.discarded += len(data)


def client_event(c, mask):
    """Service one client's readiness from a select() pass. The client may
    already have been dropped earlier in the same pass: sending the serial
    data to it failed (the peer had gone), and select() still holds the read
    event of its now-closed socket. Touching that socket again raised from
    drop() and took the whole fan-out down (2026-10-01, a probe disconnecting
    mid-stream); every client lost GNSS until systemd restarted the broker."""
    if c.dropped:
        return
    if mask & selectors.EVENT_READ:
        discard(c)
    if mask & selectors.EVENT_WRITE and not c.dropped:
        flush(c)


def open_serial():
    ser = serial.Serial(SERIAL_DEV, SERIAL_BAUD, timeout=0, exclusive=True)
    # pyserial's `exclusive` is an advisory flock; TIOCEXCL is enforced by the
    # kernel (EBUSY for any other non-root open until we close).
    fcntl.ioctl(ser.fileno(), termios.TIOCEXCL)
    sel.register(ser, selectors.EVENT_READ, "serial")
    return ser


def main():
    family = socket.AF_INET6 if ":" in BIND else socket.AF_INET
    lsock = socket.socket(family, socket.SOCK_STREAM)
    lsock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    lsock.bind((BIND, PORT))
    lsock.listen(16)
    lsock.setblocking(False)
    sel.register(lsock, selectors.EVENT_READ, "listen")
    log(f"read-only fan-out of {SERIAL_DEV} on {BIND}:{PORT}")

    # err: the last failure logged, so a retry every REOPEN_S that fails the
    # same way stays quiet. flowing: data has arrived since the open (or since
    # the last silence), so the next loss or silence is news.
    ser, next_open, err = None, 0.0, None
    last_rx, flowing, quiet = 0.0, False, False
    while True:
        now = time.monotonic()
        if ser is None and now >= next_open:
            try:
                ser = open_serial()
                last_rx, flowing, quiet = now, False, False
            except (OSError, serial.SerialException) as e:
                err = once(f"cannot open {SERIAL_DEV}: {e}; retrying every {REOPEN_S:.0f} s", err)
                next_open = now + REOPEN_S

        timeout = max(0.0, next_open - now) if ser is None else 1.0
        for key, mask in sel.select(timeout):
            if key.data == "listen":
                accept(lsock)
            elif key.data == "serial":
                try:
                    data = os.read(ser.fileno(), 65536)
                except BlockingIOError:
                    continue
                except OSError as e:
                    data, why = b"", f"read error: {e}"
                else:
                    why = "EOF (receiver unplugged or reset?)"
                if not data:
                    sel.unregister(ser)
                    ser.close()
                    ser, next_open = None, time.monotonic() + REOPEN_S
                    # After data this is news. Straight after an open it is the
                    # last failure again (a tty that opens, then hangs up, as a
                    # USB port can while it goes away), so it logs once.
                    err = once(f"lost {SERIAL_DEV}: {why}; reopening every {REOPEN_S:.0f} s",
                               None if flowing else err)
                    flowing = False
                    continue
                last_rx = time.monotonic()
                if not flowing:
                    log(f"receiving from {SERIAL_DEV} ({os.path.realpath(SERIAL_DEV)})")
                    flowing, quiet, err = True, False, None
                for c in list(clients.values()):
                    send(c, data)
            else:
                client_event(key.data, mask)

        if ser is not None and not quiet and time.monotonic() - last_rx > SILENT_S:
            log(f"no data from {SERIAL_DEV} for {SILENT_S:.0f} s "
                "(is the receiver's SBF output on this port switched off?)")
            flowing, quiet = False, True


if __name__ == "__main__":
    main()
