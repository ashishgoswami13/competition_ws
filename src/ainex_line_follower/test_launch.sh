#!/bin/bash
# Test commands for camera fix and spawn position

echo "╔═══════════════════════════════════════════════════════════════════════╗"
echo "║             LINE FOLLOWER - TEST COMMANDS                             ║"
echo "╚═══════════════════════════════════════════════════════════════════════╝"
echo ""

# Make sure we're in the workspace
cd /home/ubuntu/competition_ws
source devel/setup.bash

echo "Choose a test:"
echo ""
echo "1) Launch with default position (0, 0, 0.24)"
echo "2) Launch at position (1.0, 0.5) facing East"
echo "3) Launch at origin facing North (90°)"
echo "4) Launch at origin facing South (270°)"
echo "5) Launch paused (for manual positioning)"
echo "6) View debug image (after launching)"
echo "7) Check camera topic status"
echo "8) Monitor line detection"
echo ""
read -p "Enter choice (1-8): " choice

case $choice in
    1)
        echo "Launching with default position..."
        roslaunch ainex_line_follower line_follower_simulation.launch
        ;;
    2)
        echo "Launching at (1.0, 0.5) facing East (90°)..."
        roslaunch ainex_line_follower line_follower_simulation.launch \
            x:=1.0 y:=0.5 yaw:=1.5708
        ;;
    3)
        echo "Launching at origin facing North..."
        roslaunch ainex_line_follower line_follower_simulation.launch yaw:=0
        ;;
    4)
        echo "Launching at origin facing South (180°)..."
        roslaunch ainex_line_follower line_follower_simulation.launch yaw:=3.1416
        ;;
    5)
        echo "Launching in paused mode..."
        roslaunch ainex_line_follower line_follower_simulation.launch paused:=true
        ;;
    6)
        echo "Viewing debug image..."
        echo "Make sure simulation is running in another terminal!"
        sleep 2
        rosrun image_view image_view image:=/line_follower_node/debug_image
        ;;
    7)
        echo "Checking camera topic status..."
        echo ""
        echo "Available camera topics:"
        rostopic list | grep camera
        echo ""
        echo "Camera image_raw info:"
        rostopic info /camera/image_raw
        echo ""
        echo "Camera publishing rate (waiting 2 seconds):"
        timeout 2 rostopic hz /camera/image_raw
        ;;
    8)
        echo "Monitoring line detection..."
        echo "Press Ctrl+C to stop"
        rostopic echo /line_follower_node/line_detected
        ;;
    *)
        echo "Invalid choice!"
        exit 1
        ;;
esac
