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

The laptop holds `~/rover_live.sh` and `~/rover_live.rviz`, both pushed from this
repo — see [LAPTOP_FILES.md](LAPTOP_FILES.md). In a terminal **on the laptop's
desktop**:

```bash
bash ~/rover_live.sh
```

The script sets everything in §2 itself, so it does not matter whether this shell
has been sourced. It must be a real desktop terminal, not ssh: RViz needs a
logged-in session to render into.

From the Jetson, `./rover view` does the same thing remotely — it finds the
laptop on DHCP, pushes the current config, and refuses if nobody is logged in.
That is the normal path; the command above is for when you are sitting at the
laptop.

**Do not run `rviz2 ~/rover_live.rviz`.** The `-d` is load-bearing: `rviz2`
silently ignores a bare config path and starts with its defaults — Fixed Frame
`map`, zero displays. A blank window and no error. `rover_live.sh` exists so
this cannot happen; see [LAPTOP_FILES.md](LAPTOP_FILES.md).

### What it shows

| display | topic | |
|---|---|---|
| **ROOM (nvblox)** | `static_occupancy_grid` | the map, built as you drive |
| **Track driven** | `/fusion/path` | green line — where it has been |
| **nav2 plan** | `/plan` | amber line — the route nav2 computed |
| **Fused pose** | `/odom` | teal arrow, with its covariance ellipse |
| **Footprint** | `published_footprint` | the rover's real outline, 35 × 38 cm |
| Costmap global / local | | **off** — turn on to see obstacle inflation |
| Raw cuVSLAM | `/vo/odom` | **off** — turn on to watch it teleport |
| TF, IR left | | **off** |

**Fixed Frame is `odom`.** Not `map` — there is no map frame until something
localizes against a saved map, which is Phase 2c. Setting it to `map` gives an
empty screen and a confusing hunt.

### Driving to a goal

Use the **2D Goal Pose** tool in the toolbar: click a point and drag to set the
direction. nav2 plans a route (amber) and drives it.

### The two QoS settings that decide whether you see anything

RViz shows an empty screen for a QoS mismatch, exactly as it does for a dead
publisher, so these are worth knowing:

| topic | Durability | |
|---|---|---|
| `static_occupancy_grid` | **Volatile** | nvblox publishes volatile |
| `/global_costmap/costmap` | **Transient Local** | nav2 publishes latched |
| `/fusion/path` | **Transient Local** | so a late RViz gets the whole track |

They are opposite, and both are already set correctly in `rover_live.rviz`.

### What the nvblox mesh is not here

The 3D mesh needs `nvblox_rviz_plugin`, which is in the rover's container but
not on the laptop. Everything above uses standard ROS message types that any
Jazzy install renders, so nothing here depends on that plugin. The 2D map is
what nav2 plans on anyway.

## 5. If the laptop is not available

RViz also runs on the Jetson itself, with a monitor attached:

```bash
./rover rviz
```

That is the maintained path — it sets the environment and the `-d` for you. The
equivalent by hand, if you need to vary something:

```bash
xhost +local:docker
docker exec -it -e DISPLAY=:0 rover bash -lc \
  'source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=0; \
   unset ROS_DISCOVERY_SERVER; rviz2 -d /opt/rover/../phase2/rviz/rover_live.rviz'
```

It competes with cuVSLAM for the GPU, so expect the pose rate to dip. Fine for a
look, not for a measured run.
