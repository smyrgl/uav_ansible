#!/usr/bin/env python3
"""uav_ansible GNSS serial broker.

Single owner of one GNSS serial device, exposed as ONE bidirectional TCP port on
localhost:

  * everything read from the serial is broadcast to ALL connected clients
    (gpsd reads it for time; the Septentrio/Unicore ROS driver joins the same
    stream later, read-only)
  * everything a client sends is written to the serial (gpsd injects NTRIP RTCM
    over its read connection; the broker owns the fd so writes never interleave)

Because one process owns the serial fd, reads tee cleanly and the write path is
serialized — the thing str2str/socat can't do for a single bidirectional port.
Keep gpsd the only writer; other consumers are read-only.

Config via environment: SERIAL_DEV, SERIAL_BAUD, PORT, BIND (default 127.0.0.1 —
the port can write to the serial, so it is not exposed off-host by default).
Resilient: reopens the serial on error; drops dead clients.
"""
import os
import socket
import sys
import threading
import time

import serial  # python3-serial (pyserial)

SERIAL_DEV = os.environ.get("SERIAL_DEV", "/dev/gnss")
SERIAL_BAUD = int(os.environ.get("SERIAL_BAUD", "115200"))
PORT = int(os.environ.get("PORT", "28785"))
BIND = os.environ.get("BIND", "127.0.0.1")

_clients = set()
_clients_lock = threading.Lock()
_ser = None                      # current serial handle (None while reopening)
_ser_lock = threading.Lock()


def log(*a):
    print("[gnss-broker]", *a, file=sys.stderr, flush=True)


def _serve():
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((BIND, PORT))
    srv.listen(8)
    log(f"listening {BIND}:{PORT} (broadcast out, client writes -> serial)")
    while True:
        conn, addr = srv.accept()
        with _clients_lock:
            _clients.add(conn)
        log("client +", addr)
        threading.Thread(target=_client_reader, args=(conn, addr), daemon=True).start()


def _client_reader(conn, addr):
    """Client -> serial (e.g. gpsd's NTRIP RTCM)."""
    try:
        while True:
            data = conn.recv(4096)
            if not data:
                break
            with _ser_lock:
                if _ser is not None:
                    _ser.write(data)
    except OSError as e:
        log("client error:", addr, e)
    finally:
        with _clients_lock:
            _clients.discard(conn)
        try:
            conn.close()
        except OSError:
            pass
        log("client -", addr)


def _broadcast(data):
    dead = []
    with _clients_lock:
        for c in list(_clients):
            try:
                c.sendall(data)
            except OSError:
                dead.append(c)
        for c in dead:
            _clients.discard(c)
    for c in dead:
        try:
            c.close()
        except OSError:
            pass


def main():
    global _ser
    threading.Thread(target=_serve, daemon=True).start()
    while True:  # serial (re)open loop
        try:
            ser = serial.Serial(SERIAL_DEV, SERIAL_BAUD, timeout=1)
        except (OSError, serial.SerialException) as e:
            log(f"open {SERIAL_DEV} failed: {e}; retry in 2s")
            time.sleep(2)
            continue
        with _ser_lock:
            _ser = ser
        log(f"opened {SERIAL_DEV} @ {SERIAL_BAUD}")
        try:
            while True:
                data = ser.read(4096)
                if data:
                    _broadcast(data)
        except (OSError, serial.SerialException) as e:
            log("serial error:", e)
        finally:
            with _ser_lock:
                _ser = None
            try:
                ser.close()
            except OSError:
                pass
            time.sleep(2)


if __name__ == "__main__":
    main()
