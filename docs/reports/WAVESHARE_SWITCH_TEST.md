# Waveshare switch jumbo-frame test — run from the Mac mini

**For the Claude agent on the Mac mini.** The owner wires the cables; you set up the Mac side, run the pings,
and fill in the results at the bottom. **Every `sudo` asks for the owner's password**, so the owner types it.
The Jetson needs nothing: its port is already set up and stays that way.

## Why this test

The rover's D555 depth camera talks to the Jetson over Ethernet in **9010-byte jumbo frames** (its firmware
allows only MTU 9000). Through the **Waveshare Industrial 5P switch** (SKU TIFPS0683, RTL8367N) the camera's
small packets (IMU, discovery) arrive but no jumbo frame ever does, so there are no images. That held on two
port pairs with two proven cables, in two runs (OPEN_ISSUES.md, "wired network: the Waveshare switch", 2026-10-04).
Waveshare's wiki claims the switch forwards **9216-byte** packets.

This test checks the same thing **without the camera**: two ordinary computers, standard ping, a result the
seller can repeat. First on a direct cable (to prove the Mac and the cable can do 9000), then through the switch.

Ping with "don't fragment" is a valid test here because both ends are normal computers. (The D555's own ping
is not: it answers only up to 1472 bytes even though its data is jumbo.) **The Pi 5 cannot be used:** its
Ethernet accepts MTU 9000 but passes nothing over 1500, even on a direct cable.

## The two ends

| | Jetson (other end, nothing to do) | Mac mini (you) |
|---|---|---|
| port | `enP8p1s0`, wired | built-in Ethernet, usually `en0` (check in step 1) |
| address | **192.168.11.70/24** (static, saved) | **192.168.11.81/24** (temporary alias you add) |
| MTU | **9000** (saved) | 1500 → set **9000** temporarily |

The D555 normally sits at 192.168.11.55 on that Jetson port; **it is unplugged during this test**.
Nothing else uses 192.168.11.x. Don't change the Mac's Wi-Fi or its normal network settings.
If the Mac's Ethernet is its normal network link (it serves the rover brain's LLM on `:8080`), the
brain loses it until the cable goes back. That's fine as long as the rover isn't being driven.

## Step 1 — set up the Mac (once)

```bash
networksetup -listallhardwareports            # find "Hardware Port: Ethernet" -> its Device (e.g. en0)
networksetup -listValidMTURange en0           # must include 9000; if not, stop and report
sudo ifconfig en0 mtu 9000
sudo ifconfig en0 alias 192.168.11.81 255.255.255.0
ifconfig en0 | grep -E "mtu|inet |status"     # expect: mtu 9000, inet 192.168.11.81, status: active
/usr/libexec/ApplicationFirewall/socketfilterfw --getstealthmode   # stealth on drops pings; ask the owner to turn it off for the test
```

If `ifconfig en0 mtu 9000` is refused while the link is up, do `sudo ifconfig en0 down`, set the MTU,
then `sudo ifconfig en0 up`. If the alias then disappears (macOS can reapply the Ethernet service when
the cable is replugged), add it again. **Check `ifconfig en0` before every run.**

## Step 2 — the ping sweep (run it for each wiring below)

```bash
ifconfig en0 | grep -E "mtu|inet 192.168.11|status"
for s in 1472 2000 4000 6000 8000 8972; do
  printf "payload %-5s " $s; ping -D -s $s -c 5 -i 0.3 -t 6 192.168.11.70 | grep -E "packets transmitted"
done
```

`-D` = don't fragment. **1472** is the largest normal (MTU 1500) payload and **8972** fills a 9000 frame
exactly (9000 − 20 IP − 8 ICMP). Each ping crosses the link twice: Mac → Jetson, then Jetson → Mac. The
first packet of a run may be lost to the address lookup (ARP), so 4/5 still counts as a pass. A message like
"Message too long" means the Mac itself isn't at MTU 9000, so go back to step 1.

## Step 3 — the wirings, in this order (the owner swaps cables when you ask)

| run | wiring | what it proves |
|---|---|---|
| **A** | Jetson → **Cat6+** → Mac mini, direct | the Mac + that cable can do 9000. **If 8972 fails here, stop:** the test can't judge the switch |
| **B** | Jetson → cable → **switch port 1**, **port 2** → cable → Mac mini | the switch, first port pair |
| **C** | as B on **two other ports** (e.g. 4 and 5) | rules out one bad port |

Use two cables already proven good. The unbranded Cat6+ and the cable that came with the RealSense
both carried the camera's jumbo stream direct on 2026-10-04.
Wait about 5 s after each replug for the link, and recheck `ifconfig en0`. The switch needs power. Its
port lights are worth noting: on 2026-10-03 one port showed yellow only, the other yellow + orange.

## How to read it

| A (direct) | B / C (switch) | verdict |
|---|---|---|
| 8972 passes | 1472 passes, 2000+ fail | **the switch drops jumbo frames**: it does not do what its 9216-byte claim says. Return it with these results |
| 8972 passes | 8972 passes | the switch forwards 9000. Then the camera failure has another cause: report it and stop |
| 8972 fails | — | the Mac can't do jumbo on this cable/port. No verdict on the switch |
| 1472 fails anywhere | | a basic link/address problem, not MTU: recheck the cable, `status: active`, the alias, stealth mode |

The largest size that passes through the switch is useful too. A switch limited to standard frames passes
1472 and fails 2000.

## Step 4 — undo on the Mac (always)

```bash
sudo ifconfig en0 -alias 192.168.11.81
sudo ifconfig en0 mtu 1500
ifconfig en0 | grep mtu        # expect 1500
```

A reboot resets both too. Then put the Mac's Ethernet back the way it was. The Jetson side stays as is;
the owner plugs the D555 back into the Jetson with the Cat6+ direct (no switch). After a failed switch
run the camera may need a PoE power cycle and ~2 min of boot.

## Results (fill in)

Date / time: ______  Mac model, `en0` valid MTU range: ______  switch port lights: ______

| run | wiring | 1472 | 2000 | 4000 | 6000 | 8000 | 8972 |
|---|---|---|---|---|---|---|---|
| A | direct, Cat6+ | | | | | | |
| B | switch ports _ and _ | | | | | | |
| C | switch ports _ and _ | | | | | | |

Verdict: ______

Write the results here and in OPEN_ISSUES.md under the Waveshare section, and commit on the Mac.
Don't push unless the owner says so; this repo is also edited on the Jetson.
