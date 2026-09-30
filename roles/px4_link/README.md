# PX4 XRCE link

PX4 1.17 uses Micro XRCE-DDS Client 2.x. ROS 2 Jazzy pairs with Agent
2.4.3 and Fast DDS 2.14; Agent 3.x is incompatible with this PX4 client.
Reference: https://docs.px4.io/v1.17/en/middleware/uxrce_dds

This role builds the pinned agent against Jazzy's installed Fast DDS/Fast CDR
libraries. It installs under `/opt/uav/xrce-agent/<version>` and sets the
service library path explicitly. Other `/usr/local` libraries, including
Fast DDS 3.x used by other applications, are left intact.

Deploy with `ansible-playbook site.yml --tags px4_link` using the normal
inventory and connection settings. Builds are incremental; an existing
`/usr/local/bin/MicroXRCEAgent` no longer suppresses installation of a new
version. The service restarts only when its binary/library installation or
unit changes. The previous systemd unit is backed up before replacement.

Validate the running executable and libraries via `/proc/<MainPID>/exe`
and `/proc/<MainPID>/maps`, then check actual advancing PX4 message timestamps.
A discovered `/fmu/out` publisher is not sufficient evidence of fresh FMU data:
the agent can retain DDS entities after the XRCE client stops transmitting.
Separate raw FMU-to-agent UDP traffic (port 8888), DDS discovery, and subscriber
sample delivery when diagnosing a failure.

## Independent UART diagnostics

The Holybro baseboard connects Jetson UART1 to Pixhawk TELEM2. On the deployed
JetPack 6 system the Jetson UART is `/dev/ttyTHS1` (controller 0x3100000).
PX4 fmu-v6x maps TELEM2 to `/dev/ttyS4`.
Reference: https://docs.holybro.com/autopilot/pixhawk-baseboards/pixhawk-jetson-baseboard/mavlink-bridge

PX4's TELEM2 MAVLink instance drives this UART at 921600 baud, and since
2026-09-30 the production router owns it (`mavlink_router` role, a
`UartEndpoint`): the MAVLink shell and every other MAVLink client go through
the router's TCP port, never through the raw device, which has one reader.

## Ethernet transmit keepalive (`uav-xrce-keepalive`)

Measured on the bench, 2026-09-30, fmu-v6x with PX4 1.17, link otherwise
clean (0 of 750 pings lost, worst RTT 7 ms):

| Inbound traffic to the FMU | XRCE bursts per second | Median gap | Worst gap |
|---|---|---|---|
| none | ~10 | 66–78 ms | 375–453 ms |
| 5 Hz ping | 13 | 64 ms | 196 ms |
| 50 Hz ping | 25 | 24 ms | 34 ms |

Without inbound traffic, `/fmu/out/vehicle_attitude` and `sensor_combined`
arrived at 8.5 Hz and the DDS timesync round trip was 371 ms. The cause is in
the FMU's NuttX network stack, checked against the NuttX fork commit PX4
`main` pins (`platforms/nuttx/NuttX/nuttx`, `030417d`): `fmu-v6x` builds with
`CONFIG_NET_UDP_WRITE_BUFFERS=y` and `CONFIG_IOB_NBUFFERS=24`, so the XRCE
client's `sendto()` queues datagrams that the STM32H7 Ethernet driver only
transmits on a device poll. The driver's `stm32_txavail()` returns without
scheduling anything while its single poll work item is busy, and it has no
periodic poll, so once the 24-buffer pool is full the client blocks until the
next transmit-complete interrupt polls the queue — for example the reply to
an ICMP echo request. Related reports: PX4-Autopilot issues 26160 and 27388,
NuttX issue 11292.

The unit sends `ping -i 0.02` (about 27 kbit/s) to `px4_ip`. It is a
workaround for a driver bug, not a design feature: set
`px4_link_keepalive_interval_ms: 0` to remove it once the driver or the
board's IOB pool is fixed upstream.
