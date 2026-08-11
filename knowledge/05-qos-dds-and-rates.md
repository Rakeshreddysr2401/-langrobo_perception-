# QoS, DDS, and why we measure rates

**Used by:** every task.

> Every failure this rig has had was a **rate collapsing**, not a topic
> disappearing. That single sentence explains most of the tooling here.

---

## Topics are not function calls

A ROS topic is a **stream**, not a request. Publishers do not know or care who is
listening, and a subscriber that connects with incompatible settings simply
receives **nothing** — silently. There is no error, no exception, no log line
unless something is watching for it.

So "the topic exists" tells you almost nothing. Only the **rate** tells you the
thing is alive.

---

## QoS: the settings that make topics silently incompatible

Two matter most here.

### Reliability

| | Meaning |
|---|---|
| `RELIABLE` | retry until delivered. Nothing is lost, but a slow subscriber back-pressures the publisher. |
| `BEST_EFFORT` | send once; if it is lost, it is lost. |

Sensor streams use `BEST_EFFORT` — a dropped camera frame is irrelevant, and you
never want a slow consumer to stall the camera. Hence `input_qos: "SENSOR_DATA"`
in `config/nvblox.yaml`.

### Durability

| | Meaning |
|---|---|
| `VOLATILE` | subscribers get only what is published from now on |
| `TRANSIENT_LOCAL` | the publisher keeps the last message and **replays it to late joiners** |

`TRANSIENT_LOCAL` is for things published rarely that a late subscriber still
needs — a map, a marker, the rover's body. `src/nodes/rover_marker.py` and
`rover_trail.py` both use it, which is why RViz shows the rover immediately on
connect instead of after the next publish.

### The compatibility rule

**A subscriber may ask for weaker guarantees than the publisher offers, never
stronger.**

| Publisher | Subscriber | Result |
|---|---|---|
| RELIABLE | BEST_EFFORT | ✅ works |
| BEST_EFFORT | RELIABLE | ❌ **silence** |
| TRANSIENT_LOCAL | VOLATILE | ✅ works |
| VOLATILE | TRANSIENT_LOCAL | ❌ **silence** |

Real example from this project: reading the nvblox occupancy grid with
`TRANSIENT_LOCAL` returned nothing at all, and ROS said only:

```
New publisher discovered ... offering incompatible QoS. No messages will be
received from it. Last incompatible policy: DURABILITY
```

That warning is the only clue you get. **If a topic gives you nothing, check QoS
before anything else.**

```bash
ros2 topic info /some/topic -v     # shows QoS on both ends
```

---

## DDS discovery

ROS 2 has no master. Nodes find each other by **multicast**, and only talk if
they share a `ROS_DOMAIN_ID` (ours is `0`, set everywhere).

Two variables can silently break this, and both are cleared before every command
in `rover.sh`:

```bash
unset ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE
```

A leftover `ROS_DISCOVERY_SERVER` points discovery at a server that may not
exist, and nodes become invisible to each other while looking perfectly healthy
individually.

### The D555 is a DDS device

This camera is not a webcam on USB. It is a **network device speaking DDS** at
`192.168.11.55`, and that explains its whole failure mode:

- it **answers ping while completely dead** — ping is never a health check
- its topics are **lazily published**: nothing streams until something subscribes
- it has a fragile **control channel**, and stream start/stop happens over it

Which produces the worst trap in this project: attaching a subscriber to a raw
camera topic can take the camera **offline** at the DDS level.

```
ERROR (dds-device-impl.cpp:677) timeout waiting for reply #6294 ... "id":"hwm"
ERROR (dds-device.cpp:46)      throwing: device is offline
```

After that, no software restart recovers it — only a physical PoE power-cycle.
And because nvblox is the **only** depth subscriber, restarting nvblox cycles the
camera's stream too. Six restarts in an hour took it from 26 Hz to offline.

### NITROS

Isaac ROS negotiates a GPU-side transport for image topics. Side effect:
`ros2 topic hz` on a depth topic reports "does not appear to be published yet"
while the stack is happily measuring 20 Hz through it. **Not a fault.**

---

## Why rates, not existence

Put together: a topic can exist and deliver nothing (QoS), a node can be alive
and frozen (cuVSLAM), and a sensor can answer ping while dead (D555).

So `src/nodes/stack_status.py` measures **rates** against thresholds. Its own
header says the old version printed publisher counts and never once caught a real
failure.

`rover.sh` uses the same idea for `assert_rate`, which is how each layer refuses
to start on a broken one.

| Signal | What it proves |
|---|---|
| topic exists | almost nothing |
| publisher count ≥ 1 | a node started |
| **rate ≥ threshold** | it is actually working |
| **TF age < 0.5 s** | the pose is current, not stale |
| `/odom/health` trust | the pose is *believable*, not just fresh |

---

## Debugging checklist

| Symptom | First check |
|---|---|
| subscriber gets nothing, topic exists | **QoS mismatch** — `ros2 topic info -v` |
| node invisible to others | `ROS_DOMAIN_ID`, stale `ROS_DISCOVERY_SERVER` |
| `ros2 topic hz` empty on an image topic | NITROS — trust `./rover.sh status` |
| rate sags with low CPU load | the **camera** is dying, not the Orin |
| everything fine then all dead at once | camera went offline — power-cycle |

---

**See also:** [`02-visual-odometry.md`](02-visual-odometry.md) (silent freezing),
`FACTS.md §1`.
