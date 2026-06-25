#!/usr/bin/env python3
"""uav_ansible GNSS serial broker.

Single owner of one GNSS serial device. Two TCP services:

  READ_PORT  (broadcast): every byte read from the serial is sent to ALL
             connected clients. gpsd reads it for time; the Septentrio/Unicore
             ROS driver joins the same stream later. Read-only consumers.

  WRITE_PORT (inject):     every byte received from a client is written to the
             serial. The NTRIP client sends RTCM here. One writer in practice;
             the broker owns the fd so writes never interleave with anything.

Because one process owns the serial fd, reads tee cleanly and the write path is
serialized — the thing str2str/socat can't do for a single bidirectional port.

Config via environment: SERIAL_DEV, SERIAL_BAUD, READ_PORT, WRITE_PORT.
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
READ_PORT = int(os.environ.get("READ_PORT", "28785"))
WRITE_PORT = int(os.environ.get("WRITE_PORT", "28786"))

_read_clients = set()
_clients_lock = threading.Lock()
_ser = None                      # current serial handle (None while reopening)
_ser_lock = threading.Lock()


def log(*a):
    print("[gnss-broker]", *a, file=sys.stderr, flush=True)


def read_server():
    """Accept read-only subscribers; serial_reader broadcasts to them."""
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", READ_PORT))
    srv.listen(8)
    log(f"read/broadcast port {READ_PORT}")
    while True:
        conn, addr = srv.accept()
        with _clients_lock:
            _read_clients.add(conn)
        log("read client +", addr)


def write_server():
    """Accept RTCM/command injectors; their bytes go to the serial."""
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", WRITE_PORT))
    srv.listen(4)
    log(f"write/inject port {WRITE_PORT}")
    while True:
        conn, addr = srv.accept()
        threading.Thread(target=_write_client, args=(conn, addr), daemon=True).start()


def _write_client(conn, addr):
    log("write client +", addr)
    try:
        while True:
            data = conn.recv(4096)
            if not data:
                break
            with _ser_lock:
                if _ser is not None:
                    _ser.write(data)
    except OSError as e:
        log("write client error:", e)
    finally:
        conn.close()
        log("write client -", addr)


def _broadcast(data):
    dead = []
    with _clients_lock:
        for c in _read_clients:
            try:
                c.sendall(data)
            except OSError:
                dead.append(c)
        for c in dead:
            _read_clients.discard(c)
            try:
                c.close()
            except OSError:
                pass


def main():
    global _ser
    threading.Thread(target=read_server, daemon=True).start()
    threading.Thread(target=write_server, daemon=True).start()
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
