#!/usr/bin/env bash
# nav2_supervise.sh — run nav2; if any part of it dies, start it again whole.
# Started by ./rover nav (which kills this first). Each restart is logged to
# /tmp/nav_restarts.log; ./rover logs nav has nav2's own output.
# Why whole: nav2.launch.py (ONE DIES, ALL STOP).
# Crash traces (libbackward, nav2.launch.py BACKWARD) are copied to
# /logs/nav_crashes/ -- the repo's logs/ on the host, so they outlive the
# container and the next ./rover nav, which truncates /tmp/nav.log.
n=0
mkdir -p /logs/nav_crashes 2>/dev/null
# the rover's own costmap plugins (footprint_clear; phase3/plugins/build.sh)
PLUGINS=/opt/rover3/plugins/ws/install/setup.bash
if [ -f "$PLUGINS" ]; then source "$PLUGINS"; else echo "[nav2_supervise] WARNING: $PLUGINS missing -- run phase3/plugins/build.sh"; fi
# BRINGUP WATCHDOG (2026-09-28). Under load (Jetson ~9 on 6 cores) the
# lifecycle manager twice lost the reply to "Configuring controller_server":
# every node alive, nothing ever activated, costmaps at 0 Hz, and it stayed
# that way until someone ran ./rover nav again -- which always worked. So a
# launch that is not "Managed nodes are active" within BRINGUP_S is stopped
# here and started again, the same as a crash.
BRINGUP_S=60
while true; do
  from=$(stat -c %s /tmp/nav.log 2>/dev/null || echo 0)    # only THIS run's lines
  ros2 launch /opt/rover3/launch/nav2.launch.py &
  pid=$!
  up=""
  for _ in $(seq $BRINGUP_S); do
    sleep 1
    kill -0 "$pid" 2>/dev/null || break                      # died: handled below
    if tail -c +$((from + 1)) /tmp/nav.log 2>/dev/null | grep -q 'Managed nodes are active'; then up=1; break; fi
  done
  if [ -z "$up" ] && kill -0 "$pid" 2>/dev/null; then
    echo "$(date '+%F %T') nav2 bringup hung (not active in ${BRINGUP_S} s) -- restarting it" >> /tmp/nav_restarts.log
    echo "[nav2_supervise] bringup hung for ${BRINGUP_S} s; restarting"
    kill -INT "$pid" 2>/dev/null
    for _ in $(seq 15); do kill -0 "$pid" 2>/dev/null || break; sleep 1; done
    kill -KILL "$pid" 2>/dev/null
    pkill -KILL -f 'nav2_(controller|planner|behaviors|bt_navigator|velocity_smoother|lifecycle_manager)/' 2>/dev/null
  fi
  wait "$pid" 2>/dev/null
  n=$((n + 1))
  tail -c +$((from + 1)) /tmp/nav.log > /tmp/nav_run.log 2>/dev/null
  died=$(grep -o 'process has died.*exit code -\?[0-9]*' /tmp/nav_run.log | tail -1)
  echo "$(date '+%F %T') nav2 exited (restart $n): $died" >> /tmp/nav_restarts.log
  if grep -q 'Stack trace (most recent call last)' /tmp/nav_run.log; then
    out=/logs/nav_crashes/$(date '+%Y%m%d-%H%M%S').txt
    { echo "$died"; echo; grep -B2 -A60 'Stack trace (most recent call last)' /tmp/nav_run.log | tail -250; } > "$out" 2>/dev/null \
      && echo "$(date '+%F %T')   trace: logs/nav_crashes/$(basename "$out")" >> /tmp/nav_restarts.log
  fi
  echo "[nav2_supervise] nav2 exited; restarting in 3 s (restart $n)"
  sleep 3
done
