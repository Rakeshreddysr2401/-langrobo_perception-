# tools/net -- who sends what over DDS (read-only captures, 2026-11-15)

Raw-socket counters for the discovery / flood question (docs/OPEN_ISSUES.md, 2026-11-15, B1 and B7).
They only read packets. All but `disc.py` run on the HOST with sudo (the stack uses host networking):

| script | shows |
|---|---|
| `sudo python3 rtps_sniff.py wlP1p1s0 192.168.1.16 5` | packets from this Jetson to the Pi 5: per process, per port, per RTPS message kind |
| `sudo python3 lo_rtps.py 3 [port_lo port_hi]` | loopback traffic split into discovery (PDP, endpoint, TypeLookup) and real data |
| `sudo python3 tl_who.py 3` | which processes send / receive the TypeLookup heartbeats |
| `sudo python3 cam_tl.py` | discovery traffic between the Jetson's participants and the D555 on the camera cable |
| `python3 disc.py` (in the container, with the stack's DDS env) | how long a brand-new node takes to find /tf, the camera, /scan, /map |

Numbers of 2026-11-15 with the full stack: 26,000 packets/s on loopback, ~20,000 of them discovery.
