#!/bin/bash
# Deploy the Pi5 client to the Pi5 over WiFi. Run from the HOST (Jetson).
# The repo copy (pi5/langrobo_client.py) is the source of truth — edit there,
# deploy with this, never edit directly on the Pi5.
set -eo pipefail

PI5=${PI5:-rakhi24@192.168.1.16}
DIR=$(cd "$(dirname "$0")/.." && pwd)

ssh -o BatchMode=yes "$PI5" 'mkdir -p ~/langrobo'
scp -o BatchMode=yes "$DIR/pi5/langrobo_client.py" "$PI5:~/langrobo/"
ssh -o BatchMode=yes "$PI5" 'chmod +x ~/langrobo/langrobo_client.py'
echo "deployed to $PI5:~/langrobo/langrobo_client.py"
echo "on the Pi5:  source /opt/ros/jazzy/setup.bash && ~/langrobo/langrobo_client.py status"
