# knowledge — the concepts

Background on the ideas this rover is built from. These explain **how the
techniques work in general**; for what they measured **on this rig**, follow the
links into [PHASE1.md](../PHASE1.md).

Written before Phase 1 was built, then repointed at the code that actually
exists. Where a concept turned out differently in practice, the measured result
wins and is noted here.

| # | concept | what it covers | measured here |
|---|---|---|---|
| [01](01-frames-and-tf.md) | frames and TF | `odom`, `base_link`, optical vs REP-103, why a transform is conjugated | [ARCH §3](../ARCHITECTURE.md) |
| [02](02-visual-odometry.md) | visual odometry | feature tracking, baseline and scale, why it drifts | [PHASE1 §3](../PHASE1.md) |
| [03](03-sensor-fusion-ekf.md) | sensor fusion | how an EKF weights sensors by covariance | [ARCH §5](../ARCHITECTURE.md) |
| [04](04-tsdf-esdf-voxels.md) | TSDF / ESDF | how nvblox turns depth into a map — **Phase 2** | — |
| [05](05-qos-dds-and-rates.md) | QoS, DDS, rates | reliable vs best-effort, why a rate collapse is the signal | [TODO §1](../TODO.md) |
| [06](06-costmaps-and-inflation.md) | costmaps | how a map becomes something to plan on — **Phase 3** | — |
| [07](07-planners-and-controllers.md) | planners and controllers | the nav2 split — **Phases 3–4** | — |
| [08](08-frontier-exploration.md) | frontier exploration | how a robot decides where to look next — **Phase 2+** | — |

---

## Where practice differed from theory

**We did not build an EKF** (03). An EKF weights every sensor by covariance and
blends them all. What this rig needed was **assignment plus fallback** — each
sensor doing the one job it is good at, and being *dropped entirely* when it goes
bad. Blending a blind cuVSLAM in at low weight still made the answer worse than
using none of it: 42.4 cm against the wheels' own 17.4 cm. See
[ARCHITECTURE §5](../ARCHITECTURE.md).

**Differential-drive geometry does not apply** (01). This is a four-wheel
skid-steer, so turning drags every tyre sideways and the effective track width is
0.52 m, not the physical 0.34 m. Using the physical value made the wheels 63%
wrong on every turn. See [PHASE1 §5](../PHASE1.md).

**The accelerometer is not a position sensor** (03). Integrating it twice grows
error as t² — 180 m in a minute. Its real value is the gravity vector, which
never drifts and gives absolute roll and pitch.

**Best-effort QoS is not automatically right for high-rate telemetry** (05). It
was chosen for `/wheel_state` on exactly that reasoning and turned out not to be
the cause of anything; the real fault was `loop()` blocking in the executor. Both
theories are recorded in [TODO §1](../TODO.md), the wrong one included, so it is
not re-litigated.
