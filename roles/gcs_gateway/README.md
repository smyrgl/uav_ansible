# Egress through the Siyi GCS (gcs_gateway)

Since 2026-10-02 the Jetson has no Wi-Fi: the dongle came out when the airframe
was sealed. Its only link off the aircraft is the Siyi datalink to the UniRC 7
Pro handheld (the GCS), whose Android sits on the datalink as `192.168.144.20`
and on Wi-Fi (the house network on the bench). This role gives the Jetson
internet through the GCS: RTK corrections (drone-link to the base), apt for
converges, NTP, and Tailscale for operator access (`tailscale` role).

## Why in user space

The GCS cannot route for the drone:

- it is a production Android 13 build (`ro.build.type=user`, `ro.debuggable=0`,
  no `su`), so no iptables, NAT or policy rules can be added;
- Android tethering refuses the datalink interface ("eth0 ... is not
  tetherable"), so its own NAT never applies;
- its kernel forwards (`ip_forward=1`) and SIYI's rule 19 sends anything for
  `192.168.144.0/24` out the datalink, but packets *arriving* from the
  datalink match no rule and end at `32000: unreachable`. Tested: a Jetson
  host route via `192.168.144.20` gets "unreachable" back.

## How it works

| Piece | Where | What |
| --- | --- | --- |
| `hev-socks5-server` 2.13.1 | GCS, `/data/local/tmp/uav-gw`, Android's shell user | SOCKS5, listening on `192.168.144.20:1080` only, user `uav` + `vault_gcs_gateway_password`; a pid file makes it daemonise |
| `hev-socks5-tunnel` 2.18.0 | Jetson, `uav-gcs-tunnel.service` | TUN `gcs0` (`198.18.0.1`, MTU 1500); TCP, and UDP inside the same TCP session (`udp: tcp`), go to the server |
| `uav-gcs-tunnel-up.sh` | Jetson, run by the tunnel | `default dev gcs0 metric 100`; resolved: DNS 1.1.1.1, 8.8.8.8 on `gcs0`, all domains |
| `uav-gcs-gateway.sh` | Jetson, `uav-gcs-gateway.service` | the supervisor (below) |

The drone LAN, the PTP segment and Docker keep their own, more specific
routes, so only internet-bound traffic enters the tunnel. Name resolution
happens on the Jetson: the server is a static binary, and Android gives such
binaries no resolver, so the tunnel hands it IP addresses only. The public
resolvers also resolve the RTK base (`rtk2.puglab.io`), and work wherever the
GCS has internet, unlike the house router.

The server lives only until the GCS reboots. The supervisor checks its TCP
port every `gcs_gateway_check_interval_s` (30 s). On its own start, and
whenever the port stops answering, it connects to the GCS's network ADB
(`192.168.144.20:5555`, on at boot on this handheld), pushes the pinned binary
(only if the GCS's copy has another SHA-256), pushes the configuration, and
restarts the server. A role change to either restarts the supervisor and so
reaches the GCS. It logs state changes only:

```
SOCKS5 server on 192.168.144.20:1080 stopped answering (GCS rebooted?): reinstalling
SOCKS5 server on 192.168.144.20:1080 running (hev-socks5-server 2.13.1)
```

Both binaries are static aarch64 builds by heiher (MIT). They are fetched on
the controller into `~/.cache/uav_ansible/gcs_gateway` and checked against the
SHA-256 digests GitHub publishes for the release assets, so a Jetson without
egress can still be converged.

## One-time setup

1. `vault_gcs_gateway_password` in `group_vars/vault.yml` (16+ characters;
   see `vault.yml.example`).
2. Authorise the Jetson's ADB key on the GCS, once. On the Jetson:

   ```bash
   sudo install -d -m 0700 /var/lib/uav-gcs-gateway && sudo env HOME=/var/lib/uav-gcs-gateway ANDROID_ADB_SERVER_PORT=5038 adb connect 192.168.144.20:5555
   ```

   then, on the GCS: tick "Always allow from this computer", Allow. The
   supervisor uses the same key and its own adb server port (5038), apart from
   interactive adb use. Until then it logs the GCS's `unauthorized` state.

A Jetson with no egress at all has no `adb` yet (apt). Start the server once
from a laptop on the GCS's Wi-Fi, then converge:

```bash
adb -s <GCS Wi-Fi address>:5555 shell 'cd /data/local/tmp/uav-gw && ./hev-socks5-server server.yml'
```

That needs the binary and `server.yml` pushed there first (`adb push`); on a
GCS that has run the gateway before they are still in place.

## Measured (2026-10-02, bench, GCS on the house Wi-Fi)

| Quantity | Value |
| --- | --- |
| HTTPS request through the tunnel | 200 in 1.4 to 2.7 s |
| drone-link | QUIC to the base (`192.168.1.186:2102`) over UDP-in-TCP: handshake, "welcomed by the base", RTCM routed |
| Tailscale | direct, via the GCS's Wi-Fi address (the server's UDP relay keeps a stable mapping): 41 to 92 ms |
| NTP through the tunnel | ±445 ms: chrony rightly ignores it while PPS is selected |
| Recovery from a killed GCS server | detected after 25 s, running again 4 s later |
| `netplan apply` (networking role) | the route survives: networkd does not manage `gcs0` |

## Things to know

- **Security.** The server listens on the datalink address only, so the GCS's
  Wi-Fi neighbours cannot use it unless they route `192.168.144.0/24` to the
  GCS (Linux answers on any local address), hence the password. The Jetson's
  ADB key (root-only, `/var/lib/uav-gcs-gateway`) runs commands on the GCS as
  Android's shell user: root on the Jetson means a shell on the GCS. So does
  any laptop whose key the GCS has accepted.
- **Bandwidth.** Everything shares the Siyi datalink with the RTSP video
  (8 Mbit/s H.265). Foxglove over Tailscale with the bench layout measured
  about 8 Mbit/s out; keep the layout light in flight.
- **The GCS's Wi-Fi dozes.** Android's power saving can leave its Wi-Fi address
  briefly unanswered from the LAN ("No route to host"). The tunnel uses the
  datalink side and is unaffected.
- **macOS and adb.** An adb server left running by an earlier app session can
  lose macOS's Local Network permission and fail with "No route to host" while
  `nc` connects fine; `adb kill-server` and connect again.
- **Metric 100** wins over a DHCP Wi-Fi default route (600). If Wi-Fi comes
  back, raise `gcs_gateway_route_metric` or set `gcs_gateway_state: absent`.
- `gcs_gateway_state: absent` removes the units, binaries and configuration.
  The supervisor's ADB key stays, so a later return needs no new approval.
