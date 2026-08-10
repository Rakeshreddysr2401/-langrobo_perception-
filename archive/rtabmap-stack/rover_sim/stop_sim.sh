#!/bin/bash
# Stop rover_sim on the laptop.
pkill -f "contract_bridge.py" 2>/dev/null
pkill -f "restamper/build/restamper" 2>/dev/null
pkill -f "parameter_bridge" 2>/dev/null
pkill -f "gz sim" 2>/dev/null
pkill -9 -f "ruby.*gz sim" 2>/dev/null
echo "rover_sim stopped"
