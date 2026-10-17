#!/usr/bin/env bash
# Install the rover's systemd units.   sudo ops/install_services.sh [--boot-up]
#   rover-watchdog   always installed + enabled (observe only: restarts nothing)
#   rover-up         only with --boot-up: ./rover up at every boot. The rover
#                    then starts its sensors and links with nobody watching;
#                    it does not drive by itself, but decide that deliberately.
set -e
here=$(cd "$(dirname "$0")" && pwd)
install -m 644 "$here/rover-watchdog.service" /etc/systemd/system/
install -m 644 "$here/rover-up.service" /etc/systemd/system/
systemctl daemon-reload
systemctl enable --now rover-watchdog
[ "${1:-}" = --boot-up ] && systemctl enable rover-up && echo "rover-up enabled: the stack starts at boot"
systemctl --no-pager status rover-watchdog | head -5
