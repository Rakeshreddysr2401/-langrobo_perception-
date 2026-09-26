#!/usr/bin/env bash
# nav2_supervise.sh — run nav2; if any part of it dies, start it again whole.
# Started by ./rover nav (which kills this first). Each restart is logged to
# /tmp/nav_restarts.log; ./rover logs nav has nav2's own output.
# Why whole: nav2.launch.py (ONE DIES, ALL STOP).
n=0
while true; do
  ros2 launch /opt/rover3/launch/nav2.launch.py
  n=$((n + 1))
  echo "$(date '+%F %T') nav2 exited (restart $n): $(grep -o 'process has died.*exit code -\?[0-9]*' /tmp/nav.log | tail -1)" >> /tmp/nav_restarts.log
  echo "[nav2_supervise] nav2 exited; restarting in 3 s (restart $n)"
  sleep 3
done
