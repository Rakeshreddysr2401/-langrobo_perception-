# FACTS — things we know because we measured them

**This is the most valuable file in the repo.** Every line cost at least one
session to find, and several cost a whole day. The code is replaceable; this is
not.

Rules for this file:

- Only things that were **measured**, with the number and the date.
- If something is believed but not measured, it goes in `TODO.md`, not here.
- When a fact is superseded, **strike it through and say what replaced it** —
  never silently delete it, because the old belief explains old code.

---

## 1. The camera (Intel RealSense D555)

### It is a network device, not a USB device

It speaks **DDS over ethernet/PoE at `192.168.11.55`**. This changes everything
about how it fails.

### Ping is not a health check

The camera answers ping perfectly while completely dead. The only real check is
whether `infra1/image_rect_raw` has a live publisher. That is what
`nodes/stack_status.py` and the `cam_ready()` helper actually test.

### The IR emitter must stay OFF — measured 2026-08-09

| emitter | real push | `/odom` reported | error |
|---|---|---|---|
| ON | 100 cm | **26 cm** | ~4x under |
| OFF | 100 cm | **97 cm** | 3% |

The projector is rigidly mounted on the camera, so its dot pattern is repainted
from the camera's own viewpoint every frame. On a low-texture floor the dots stay
almost stationary in the image while the rig translates, so cuVSLAM — which
tracks features in infra1/infra2 — sees almost no motion.

Passive-stereo **depth stays metric-correct either way** (verified: 140 cm wall
reads 1.40 m), so nvblox is fine. Emitter-off depth is just noisier on
textureless surfaces. Localization matters more than pretty depth.

### `enable_sync:=false` is mandatory — measured 2026-07-22

With the camera's cross-stream frame syncer ON, enabling colour gates the
infra1/infra2 pair behind colour alignment over DDS and **starves IR to under
1 Hz**. cuVSLAM then stalls, there is no `/odom`, no TF, and RViz goes blank.

It is **not** a bandwidth problem — even 424x240x6 colour starved it — and no
camera power-cycle fixes it. With sync OFF everything coexists: IR ~22-23 Hz,
`/odom` ~18 Hz, colour ~7 Hz.

### Subscribing to a raw camera topic can kill the camera — measured 2026-08-11

Killed the camera **twice in one day**: once by toggling the RViz camera Image
display, once by a throwaway script that subscribed to `depth/image_rect_raw`
and `color/image_raw` to grab one frame.

Those topics are **lazily published** — nothing streams until something
subscribes. A new subscriber makes the driver start/stop streams over the
camera's DDS control channel, and that channel is fragile:

```
ERROR (dds-device-impl.cpp:677) timeout waiting for reply #6294 ... "id":"hwm"
ERROR (dds-device.cpp:46)      throwing: device is offline
```

`hwm` = hardware monitor. After `device is offline`, **no software restart
recovers it.** Bring-up aborts with "D555 is NOT streaming after 40s" and only a
**physical PoE power-cycle** (unplug ~5 s) brings it back.

### Restarts are not free either — measured 2026-08-11

nvblox is the **only** depth subscriber. So every restart of nvblox drops the
subscriber count to zero and stops/restarts the stream over that same fragile
channel.

About six `up`/`fuse`/`cam` cycles in one hour took the camera from 26 Hz to
**9.8 Hz** (`trust=False`, `CAMERA_STARVED`) at a load average of only **3.1** —
so the Orin was *not* the bottleneck — and then it went offline entirely.

Killing `nvblox_node` on its own is worse: depth stays at 0 Hz permanently and
nvblox logs `Last view not set for sensor type. Decaying all voxels`.

**Bring the stack up once, then work. Batch config changes into one restart.**

### Depth topics are NITROS-negotiated

`ros2 topic hz` on `depth/image_rect_raw` reports "does not appear to be
published yet" even while the stack is measuring 20 Hz through it. That is
Isaac's GPU-side type negotiation, **not** a fault. Do not chase it.

---

## 2. Geometry

### Camera height is 0.163 m, not 0.200 — measured 2026-08-10

`floor_probe.py` deprojects the depth centre column and asks where open floor
lands. `base_link`'s origin is on the ground, so floor must read z ≈ 0.000.

| camera z in TF | floor reads |
|---|---|
| 0.200 (guessed) | **+0.037 m** |
| **0.163 (measured)** | **+0.000 m** (16-84%: -0.002 .. +0.004, 19.5k points) |

A 3.7 cm error was enough to make the **floor map as an obstacle**, which
produced a solid inflation plateau across the full ±1 m width starting ~0.25 m
ahead, with no lethal cells anywhere. nav2's MPPI then crawled at 0.082 m/s
(against `vx_max` 0.30), the collision monitor's SlowZone cut that to 0.033 m/s,
the rover covered **0.9 cm in 30 s**, and nav2 aborted on "Failed to make
progress".

There is a residual **+1.07° nose-up pitch**, which lifts floor to ~+0.009 m at
1.25 m range.

**Re-run `floor_probe.py` after any camera remount.**

### Chassis

L 0.30 m × W 0.17 m × H 0.16 m. Camera mount x=0.10, z=0.163.
Wheels 85 mm. `WHEEL_BASE_M = 0.34` (matches the firmware).

### The camera looks forward only, ~87°

There is no pan/tilt head. The rover is **blind to its sides and behind**. A
wedge of map in front of it is the *correct* picture from a single viewpoint; a
room only appears if you drive around. This is hardware, not a bug, and it
explains most "why is the map wrong" confusion.

---

## 3. Odometry and pose

### cuVSLAM freezes silently

If the camera rate drops below ~10 Hz, cuVSLAM stops producing odometry **and
never recovers** — while still reporting `slam_pose_ok: true`. It does not raise
an error. This is why `stack_status.py` measures **rates**, not publisher counts:
every failure this rig has had was a rate collapsing, not a topic disappearing.

### The accelerometer is deliberately not fused

A MEMS accel fused naively diverges by hundreds of metres — gravity and bias
swamp the real signal. A slow ground robot gains nothing and inherits all the
drift. `config/ekf.yaml` fuses **gyro yaw-rate only**.

### `two_d_mode: true` is load-bearing — measured 2026-08-11

Running visual-only (no EKF), `/odom` reported **z = -0.272 m** — the rover
believed it had sunk 27 cm through the floor. nvblox slices obstacles in a fixed
height band, so a sinking pose drags that band through the floor and maps floor
as wall. With the EKF running, `/odometry/filtered` reports z = **exactly 0.0**.

### The pose is rock solid standing still — measured 2026-08-11

160 s parked: translation drift **0.0000 m**, yaw wobble **±0.03°**.

---

## 4. Mapping (nvblox)

### The defaults delete the map behind you — measured 2026-08-11

Two independent mechanisms in `nvblox_base.yaml`:

1. **TSDF decay.** `decay_tsdf_rate_hz 5.0`, `tsdf_decay_factor 0.95`,
   `decay_integrator_deallocate_decayed_blocks true`. Every voxel's weight is
   multiplied by 0.95 five times a second; below `tsdf_decayed_weight_threshold`
   (0.001) the block is deallocated. From `projective_integrator_max_weight 5.0`
   that is `ln(0.001/5)/ln(0.95)` = 166 steps = **~33 seconds**.
2. **`map_clearing_radius_m: 7.0`** at 1 Hz — everything beyond 7 m is deleted.

With an 87° forward-only camera, that guarantees a torch beam and never a room.
Both are neutralised in `config/nvblox.yaml`.

**`tsdf_decay_factor` must be strictly < 1.0.** `tsdf_decay_integrator.cu:27`
asserts `value < 1.0` and hard-aborts the whole node on exactly 1.0
(tried it: `Check failed: value < 1.0 (1 vs. 1)`, node never came up). `0.999999`
at 5 Hz takes ~20 days to decay a voxel, i.e. never.

**Cost of this:** the map now never forgets. Moving objects leave permanent
ghosts and pose drift smears walls instead of letting them fade.

### Slice heights are read at init, not at runtime — measured 2026-08-11

`ros2 param set /nvblox_node static_mapper.esdf_slice_min_height` reports
"Set parameter successful" and **changes nothing**. Verified by getting a
byte-identical map back afterwards. You must edit the YAML and restart.

### The over-thick map is NOT the floor — measured 2026-08-11

Same stationary viewpoint, freshly restarted at each band:

| band | free | occupied | **occ/free** | coverage |
|---|---|---|---|---|
| 0.12 – 0.40 | 1112 | 520 | **0.47** | 5.0 × 6.9 m |
| 0.35 – 0.60 | 653 | 315 | **0.48** | 2.5 × 4.4 m |

The ratio is unchanged. Lifting the band 23 cm clear of the floor bought nothing
and cost half the coverage. So the floor is **not** what fills the map, and the
0.12 workaround can come down (it was only ever dodging the camera-height error,
which is now fixed).

### Noise accumulation is real but small — measured 2026-08-11

Parked 5.5 minutes, nothing in the room moving: occupied 520 → 549, free flat.
About **5 cells/minute**. Reaching the bad map's 2850 would take over 7 hours.

### A parked rover maps CLEANLY; a driven one smeared

| | free | occupied | occ/free |
|---|---|---|---|
| parked | 1112 | 520 | **0.46** ← healthy, thin edges |
| driven | 1721 | 2850 | **1.66** ← blob |

Arithmetic: a far wall at 4 m across an 87° cone is an arc ~6.1 m long. One 5 cm
cell thick, that is ~122 occupied cells. **2850 were measured** — the "wall"
averages about a metre thick.

Since the floor, accumulation and parked drift are all ruled out, the remaining
suspect is **pose error while moving**. See `TODO.md`.

---

## 5. Networking and the view

| Thing | Address | Note |
|---|---|---|
| RViz laptop | **192.168.1.10** | Docs used to say `.12`; DHCP gave `.12` to the ESP32 |
| ESP32 | `rover-esp32.local` | Resolve it; never trust a typed IP |
| D555 | 192.168.11.55 | separate subnet |
| Pi 5 | 192.168.1.16 | teleop web on :8091 |

Everything on this LAN is DHCP. **Resolve before trusting any hardcoded IP.**

### RViz fails silently, and the evidence lands on the other machine

Four independent causes, all measured 2026-08-10/11:

1. **Wrong IP** — `.12` is the ESP32, which refuses ssh and looks like a laptop
   that is switched off.
2. **Nobody logged in** — at the GDM login screen, RViz launched over ssh runs
   invisibly and reports no error.
3. **Wayland** — `loginctl -p Display` is **empty** on a Wayland session. The
   authoritative source for both display and cookie is the Xwayland process
   cmdline: `pgrep -a Xwayland` → `:0 ... -auth /run/user/1000/.mutter-Xwaylandauth.XXXXXX`.
   That cookie path is regenerated on every login, so it can never be hardcoded.
4. **Xwayland not started yet** — GNOME starts it **on demand**, so right after
   login there is no X server at all. Never fall back to guessing `:0`.

The error always lands in `/tmp/rviz.log` **on the laptop**.

### Wifi power-save breaks single pings — measured 2026-08-11

The laptop's radio sleeps; RTT swings 21 → 128 ms and the first ICMP packet is
dropped. A single `ping -c1` reports a healthy, logged-in laptop as "off" while
ssh to it works fine. **Always retry.**

### The wifi channel is nearly saturated — measured 2026-08-11

Enabling the raw camera image display took the nvblox map arriving at the laptop
from **9.4 Hz to zero**, and stretched `/odom` gaps to 0.47 s. Turning it off
restored 9.3 Hz immediately.

Budget: the map alone is ~290 kB/s. Anything new on this link must be measured,
not assumed.

---

## 6. Drivetrain

### The firmware publishes unconditionally at 20 Hz when connected

`rover_firmware_v2.ino:391` — `EXECUTE_EVERY_N_MS(50, ...)` once
`AGENT_CONNECTED`. **No motion gating.** So a low `/wheel_state` rate can never
be explained by "the rover was parked".

The only other 1000 ms timer in that file is the `WAITING_AGENT` ping at `:374`.
So a rock-steady **1.00 Hz means the ESP32 is not in the connected loop at all** —
a different failure from the ~10 Hz half-rate seen on 2026-08-10.

### Hand-pushing is safe

`pidStep()` returns 0 whenever target velocity is < 0.01, and `driveSide()` with
zero duty pulls both BTS7960 pins low — that is **coast, not brake**. The wheels
free-wheel and the PID does not fight you. *Re-check if the firmware changes: a
PID that actively held zero would resist.*

### There is no flashing toolchain on the Jetson or the Pi 5

No arduino-cli, no pio, no esptool, no ESP32 cores, no USB serial. The repo's
`WIFI_PASS` is a placeholder. **Any flash must be done by you**, from a machine
with the real credentials, targeting `rover-esp32.local`.

---

## 7. Environment

The whole stack runs in **one Docker container** built from the old repo's
Dockerfile. The code directory is bind-mounted **read-only** over the baked copy,
so editing a config or node and restarting takes effect with **no rebuild**.

### The image has no build recipe and cannot be reproduced — established 2026-08-11

`rover.sh` runs `orin-nav:1.1`, image ID
**`sha256:2a3d7f1d30dde60397899073ff8d291dbb75b57a0a042621aee5e84640e3741d`**,
built 2026-07-19 16:35 (+05:30). Pin the ID, not the tag: `orin-nav:0.0.2` also
exists, is also 57.8 GB, and is a **different image** (`e1c32d08f6c6`).

The old repo's `orin-nav-stack/Dockerfile` — now copied to `docker/Dockerfile` —
is fifteen lines of `COPY` on top of `FROM isaac_ros:cuvslam-unified`. Tracing
that base with `docker history` and the image labels:

```
orin-nav:1.1                                 ← the only layer with a Dockerfile
  └─ isaac_ros:cuvslam-unified   57.8 GB     ← no Dockerfile exists, anywhere
       └─ isaac_ros:langrobo-prod 54.4 GB    ← no Dockerfile exists, anywhere
```

`orin-nav:1.1` carries `com.docker.compose.project=robot` and
`...config_files=/home/rakhi24/robot/docker-compose.yml` labels. Those are the
fingerprint of an image made by **`docker commit` on a running container**, not
by a build — so the 57.8 GB base was never described by a recipe and cannot be
reproduced from one.

**Consequence:** the only real backup is `docker save`, to external storage —
`/` has 45 GB free of 227 GB, which is not enough room to be casual about it. See
`TODO.md §13`.

`$HOME/orin-nav-stack` is a **symlink** to `langrobo_perception/orin-nav-stack`,
and that symlink is what the container actually mounts. Do not delete it.
