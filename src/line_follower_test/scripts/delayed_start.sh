#!/bin/bash
# Delayed startup script for line follower
# This ensures robot spawns properly before other nodes start

echo "================================================"
echo "Waiting for robot to spawn and stabilize..."
echo "================================================"

# Wait for Gazebo to be ready
sleep 3

echo "Unpausing Gazebo physics..."
rosservice call /gazebo/unpause_physics

echo "Waiting 2 more seconds for physics to stabilize..."
sleep 2

echo "================================================"
echo "Robot spawned and stabilized successfully!"
echo "Line follower will now start..."
echo "================================================"
