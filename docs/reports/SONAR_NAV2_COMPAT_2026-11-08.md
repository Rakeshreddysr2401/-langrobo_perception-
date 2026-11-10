# Nav2 sonar no-return compatibility — November8,2026

Real stationary bring-up exposed repeated `sonar_right: Malformed range message`.
The installed Nav21.5.0 Lyrical consumer rejects positive infinity in its Range
validator; the Pi publishes REP117 +inf for no echo. A later no-echo observation
therefore leaves the preceding finite echo latched. The actual installed consumer
reproduced this in ROS domain42, without any output connected to wheels.

`nav/nodes/sonar_nav2_compat.py` preserves raw `/sonar/*` and all timestamps,
frames, finite echoes and metadata. Only valid positive-infinity no-return values
become max_range on private `/nav2_sonar/*` inputs to collision_monitor. This
conservatively inserts a distant arc (currently4m), rather than claiming a finite
nearby return or bypassing obstacle checks. Invalid values/metadata remain invalid.
The launch supervises this bridge with the rest of Nav2; its exit stops that launch.
No command caps, physical geometry, margins, source timeout or model was changed.
Remove this compatibility bridge when the installed Nav2 supports no-return values.

Validation:3 pure regression tests passed. Actual installed collision_monitor was
then tested through the full ROS bridge on domain42/private command outputs:
far->pass(.1m/s), near->stop, no echo->pass versus original retaining stop,
near again->stop, NaN after near->stop, far again->pass. Each phase required at
least5 output samples after settling. This is a consumer regression, not a driving
trial. Scripts: nodes/test_sonar_nav2_compat.py, tools/sonar_nav2_probe.py; the latter
requires ROS_DOMAIN_ID=42 and RANGE_PROBE_USE_BRIDGE=1 for the bridge path.

After real Nav2 restart:all6 lifecycle nodes active; no malformed-range errors.
A subscribe-only stationary check matched192 finite-left and191 no-echo-right
raw/private pairs with zero metadata/value mismatches. Estimated pose span over
that observation was0.985mm/0.101deg; there was no physical ground-truth reference.
30s health:odom20Hz, scan10Hz, camera IR/depth30Hz, map.52Hz, cleaned grid4.87Hz,
global/local costmaps.74/1.65Hz. No nonzero command was observed; no real goal or
velocity was sent. Wheel link19.9Hz and ESP32 command subscription passed; motor
power, stopping distance and movement accuracy still require an owner floor test.

The full Isaac contract rerun could not pass because the simulator/twin were down
following metadata-discovery failures. All13 inputs were absent on domain42.
Saved-snapshot parity passed, but those snapshots are October19/20 and do not
validate this new change. Isolated Nav2 regression and real stationary evidence
must not be presented as a completed full-stack motion qualification.

Real actor remains v002 (SHA2566f5add582e4c7355781600d8e994d0974403e7967b64ba1d7ed75adab007989b),
REACH_RL=1 and REACH_CAMERA_HANDOVER=0. Every new actor remains unqualified.
Raw logs and source/evidence copies: laptop kino_sim/build/overnight_20261107/.
Upstream reference: https://github.com/ros-navigation/navigation2/blob/1.5.0/nav2_collision_monitor/src/range.cpp
Installed headers and binary were checked directly; no package was upgraded.
