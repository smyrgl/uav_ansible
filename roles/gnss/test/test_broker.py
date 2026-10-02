"""Regression tests for the GNSS fan-out's client handling.

Run from this directory: python3 -m unittest test_broker -v
pyserial is stubbed, so the tests run anywhere (no receiver needed).
"""
import importlib.util
import pathlib
import selectors
import socket
import sys
import types
import unittest

if "serial" not in sys.modules:
    stub = types.ModuleType("serial")
    stub.SerialException = OSError
    stub.Serial = object
    sys.modules["serial"] = stub

_path = pathlib.Path(__file__).resolve().parent.parent / "files" / "uav-gnss-broker.py"
_spec = importlib.util.spec_from_file_location("uav_gnss_broker", _path)
broker = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(broker)
broker.log = lambda *a: None


class ClientHandling(unittest.TestCase):
    def setUp(self):
        broker.sel = selectors.DefaultSelector()
        broker.clients.clear()

    def tearDown(self):
        broker.sel.close()

    def connect(self):
        ours, peer = socket.socketpair()
        ours.setblocking(False)
        c = broker.Client(ours, ("127.0.0.1", 40000))
        broker.clients[ours] = c
        broker.sel.register(ours, selectors.EVENT_READ, c)
        return c, peer

    def test_stale_event_after_failed_send_is_ignored(self):
        # The 2026-10-01 crash: the peer goes away, fanning out serial data to
        # it fails and drops it, then the same select() pass delivers the read
        # event of its closed socket.
        c, peer = self.connect()
        peer.close()
        broker.send(c, b"\x24\x40" + bytes(4094))
        self.assertTrue(c.dropped)
        self.assertNotIn(c.sock, broker.clients)
        broker.client_event(c, selectors.EVENT_READ | selectors.EVENT_WRITE)  # must not raise

    def test_drop_is_idempotent(self):
        c, peer = self.connect()
        broker.drop(c, "first")
        broker.drop(c, "second")  # must not raise on the closed socket
        self.assertEqual(broker.clients, {})
        peer.close()

    def test_other_clients_survive_one_disconnect(self):
        gone, gone_peer = self.connect()
        live, live_peer = self.connect()
        gone_peer.close()
        for c in list(broker.clients.values()):
            broker.send(c, b"SBF")
        broker.client_event(gone, selectors.EVENT_READ)
        self.assertEqual(list(broker.clients.values()), [live])
        self.assertEqual(live_peer.recv(16), b"SBF")
        live_peer.close()

    def test_client_writes_are_discarded_not_fatal(self):
        c, peer = self.connect()
        peer.sendall(b"SSSSSSSSSS\n")
        broker.client_event(c, selectors.EVENT_READ)
        self.assertFalse(c.dropped)
        self.assertEqual(c.discarded, 11)
        peer.close()
        broker.client_event(c, selectors.EVENT_READ)  # EOF -> dropped once
        self.assertTrue(c.dropped)


if __name__ == "__main__":
    unittest.main()
