#!/bin/bash
# Deploy rover_sim to the laptop over SSH. Run from the Jetson HOST.
# The repo copy (rover_sim/ + config/d555_contract/) is the source of truth —
# edit there, deploy with this, never edit on the laptop (same rule as pi5/).
set -eo pipefail

LAPTOP=${LAPTOP:-rakhi24@192.168.1.12}
DIR=$(cd "$(dirname "$0")/.." && pwd)

ssh -o BatchMode=yes "$LAPTOP" 'mkdir -p ~/langrobo'
rsync -a --delete "$DIR/rover_sim/" "$LAPTOP:~/langrobo/rover_sim/"
rsync -a --delete "$DIR/config/d555_contract/" "$LAPTOP:~/langrobo/rover_sim/d555_contract/"
ssh -o BatchMode=yes "$LAPTOP" 'chmod +x ~/langrobo/rover_sim/*.sh ~/langrobo/rover_sim/bridge/contract_bridge.py'
echo "deployed to $LAPTOP:~/langrobo/rover_sim/"
echo "start:  ssh $LAPTOP '~/langrobo/rover_sim/run_sim.sh'"
