---
name: rover-stop
description: Stop the LangRobo rover safely — e-stop levels, optional map save, laptop RViz cleanup, what to leave running (Pi5 brain/Telegram). Use when the user says "stop the rover", "shut it down", "e-stop", or similar.
---

# Rover stop / shutdown (run from the Jetson)

## 0. If this is an EMERGENCY (rover moving badly)

```bash
cd ~/orin-nav-stack && ./run_stack.sh stop     # kills all motion nodes + publishes zeros
# in doubt / container weird:
./run_stack.sh down                            # removes container; ESP32 watchdog halts wheels ≤0.5 s
# container already down but wheels alive:
docker run --rm --network host --entrypoint bash orin-nav:1.1 -lc \
  'source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=0; ros2 topic pub -r 20 -t 60 /cmd_vel geometry_msgs/msg/Twist "{}"'
```

## 1. Normal stop sequence

1. **Save the map first if this session mapped anything worth keeping**:
   ```bash
   docker exec orin_nav bash -lc 'source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=0; ros2 service call /slam/save_map std_srvs/srv/Trigger'
   ```
   Lands on the host at `~/orin-nav-stack/maps/current` (gitignored, persists).
2. Stop motion + nav: `./run_stack.sh stop` (nav2/deadband/guard killed, zeros held).
3. Full stack off: `./run_stack.sh down` (this IS the e-stop state per house rules —
   container down = rover cannot move).

## 2. Laptop RViz cleanup

```bash
ssh rakhi24@192.168.1.10 'pkill -x rviz2'      # laptop is .10 — .12 is the ESP32 (DHCP swap 2026-08-10)
                                               # -x exact; pgrep/pkill -f self-matches over ssh
```

## 3. What to LEAVE RUNNING

- **Pi5 brain + micro-ROS** (`langrobo-brain`, `langrobo-microros`): leave up — Telegram
  keeps working (look() goes dark without the Jetson look feed; that's expected).
  Fully stopping them needs sudo on the Pi5 (not passwordless) — usually don't.
- ESP32 just idles when /cmd_vel goes quiet (500 ms watchdog). No action needed.
- Mac VLM: leave alone.

## 4. Restart later

Use /rover-start (or `./run_stack.sh up` then `nav2`, `vision`; verify with
`./run_stack.sh status`). Two things that often need a **physical power-cycle**:
- **ESP32 wheels** — if the Pi5 or micro-ROS agent restarted while the rover was off,
  the ESP32 won't reconnect (until firmware `e282a13` is flashed). `/cmd_vel` subs = 0.
- **D555 camera** — if it logged `device is offline` (ethernet/PoE DDS drop), it pings
  but won't stream. Power-cycle its PoE cable, then `./run_stack.sh cam`. See /rover-start §1a.
