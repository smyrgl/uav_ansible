# Tailscale (tailscale)

Operator access to the Jetson from the tailnet, over whatever egress it has.
Since 2026-10-02 that is the Siyi GCS (`gcs_gateway` role): the Jetson has no
Wi-Fi of its own, so nothing on the house network can reach it directly.

- Package `tailscale` from Tailscale's apt repository. The repository key is
  checked against a pinned fingerprint (`tailscale_key_fingerprint`,
  `2596A99EAAB33821893C0A79458CA832957F5868`, Tailscale Inc. package
  repository signing key, RSA 4096 from 2020) before it is trusted.
- Flags (`tailscale_flags`): `--accept-dns=false`, so the Jetson keeps its own
  resolver (resolved, with DNS on `gcs0`); `--accept-routes=false`;
  `--operator=<target_user>`, so `tailscale` runs without sudo.
- Login, once per node: with `vault_tailscale_authkey` set, the role joins on
  its own (prefer a pre-approved, tagged key). Without one it prints the
  command; `sudo tailscale up --hostname=jethawk --accept-dns=false
  --accept-routes=false --operator=john` prints a link to approve. A node that
  is already running only gets its preferences reasserted (`tailscale set`).

## Reaching the Jetson

The node is `jethawk`, `jethawk.tailbfa4a.ts.net`, `100.79.171.116`:

| Use | Address |
| --- | --- |
| SSH | `ssh -o HostKeyAlias=jethawk john@jethawk.tailbfa4a.ts.net` (the alias reuses the known host key) |
| Ansible | `-i jethawk.tailbfa4a.ts.net,` (no task depends on the inventory name) |
| Foxglove | `ws://jethawk.tailbfa4a.ts.net:8765` |
| QGC on a laptop | TCP `jethawk.tailbfa4a.ts.net:5760` |

Measured 2026-10-02: a direct path through the GCS (its Wi-Fi address and the
gateway server's UDP relay port), 41 to 92 ms, DERP (`den`) as the fallback;
SSH command round trip 1.2 s. All of it rides the Siyi datalink, next to the
RTSP video.

## Not done, on purpose

- No subnet route for the drone LAN: `--advertise-routes=192.168.144.0/24`
  would put PX4, the D555 and the Avia on the tailnet. Add it, and approve it
  in the admin console, only if that is wanted.
- Node keys expire (180 days by default). For an aircraft, disabling key
  expiry for this node in the admin console avoids a lockout in the field;
  that is the tailnet owner's call.
