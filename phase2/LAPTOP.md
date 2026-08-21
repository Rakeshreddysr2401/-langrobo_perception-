# Watching the rover from a laptop

RViz on the laptop, everything else on the Jetson. Nothing about the rover
changes — RViz only subscribes.

---

## 1. Requirements

- **ROS 2 Jazzy** on the laptop (Ubuntu 24.04). Other distros will mostly
  interoperate over DDS but message definitions can differ; Jazzy avoids that.
- **Same WiFi** as the rover — `Airtel_Singireddy's`, the `192.168.1.x` network.

## 2. Every session, on the laptop

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
unset ROS_DISCOVERY_SERVER          # a stale one makes nodes silently invisible
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

`ROS_DOMAIN_ID=0` and clearing `ROS_DISCOVERY_SERVER` are both load-bearing. The
Jetson and the Pi 5 use plain multicast on domain 0; a laptop pointed at a
discovery server sees an empty graph and looks exactly like a network fault.

## 3. Check it can see the rover

```bash
ros2 topic list | grep -E '/odom|/fusion|/vo'
ros2 topic hz /odom          # want ~20 Hz
```

If the list is empty, in order: same WiFi? `ROS_DOMAIN_ID=0`?
`ROS_DISCOVERY_SERVER` unset? Some routers block multicast between wireless
clients — check whether the Jetson can be pinged at `192.168.1.15`.

## 4. Run RViz

Copy `phase2/rviz/rover.rviz` from this repo to the laptop, then:

```bash
rviz2 -d rover.rviz
```

### What it shows

| display | topic | |
|---|---|---|
| **Fused pose** | `/odom` | teal arrow — where the rover thinks it is, with its covariance ellipse |
| **Where it has been** | `/fusion/path` | green line — the track it has driven |
| **TF** | | the frame tree: `odom → base_link → camera0_link` |
| Raw cuVSLAM | `/vo/odom` | red arrow, **off by default**. Turn it on to watch it teleport while the fused pose does not |
| IR left | `infra1` | off by default; on if you want to see what the camera sees |
| nvblox mesh / 2D slice | | the map, once Phase 2b is running |

**Fixed Frame is `odom`.** Not `map` — there is no map frame until a map is being
saved and localized against, which is Phase 2c.

The **covariance ellipse** around the pose grows when a sensor drops out. That is
the estimate telling you it is less sure, and it is worth watching: it should
swell when you drive at a blank wall and shrink again afterwards.

## 5. If the laptop is not available

RViz also runs on the Jetson itself, with a monitor attached:

```bash
xhost +local:docker
docker exec -it -e DISPLAY=:0 rover bash -lc \
  'source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=0; \
   unset ROS_DISCOVERY_SERVER; rviz2 -d /opt/rover/../phase2/rviz/rover.rviz'
```

It competes with cuVSLAM for the GPU, so expect the pose rate to dip. Fine for a
look, not for a measured run.
