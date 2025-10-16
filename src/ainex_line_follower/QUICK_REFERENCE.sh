#!/bin/bash
# Quick Reference Guide for Line Follower

cat << 'EOF'
╔════════════════════════════════════════════════════════════════════╗
║          AINEX LINE FOLLOWER - QUICK REFERENCE GUIDE               ║
╚════════════════════════════════════════════════════════════════════╝

✅ FIXES APPLIED
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
1. Robot head now tilts downward (head_tilt = 0.5 rad)
2. Camera points at ground (30° downward pitch)
3. Head controllers initialized in line_follower_node.py
4. Debug image published correctly to /line_follower_node/debug_image

📋 LAUNCH COMMANDS
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# Full simulation (recommended)
roslaunch ainex_line_follower line_follower_simulation.launch

# Line follower only (robot already spawned)
roslaunch ainex_line_follower line_follower_only.launch

# Test mode (simulated camera, no Gazebo)
roslaunch ainex_line_follower test_line_follower.launch

🎥 VIEW CAMERA FEEDS
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# Raw camera feed
rosrun image_view image_view image:=/camera/rgb/image_raw

# Debug image with line detection overlay
rosrun image_view image_view image:=/line_follower_node/debug_image

# View in RViz
rosrun rviz rviz
  → Add → By topic → /line_follower_node/debug_image → Image

🔍 MONITORING
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# Check line detection status
rostopic echo /line_follower_node/line_detected

# Monitor camera rate
rostopic hz /camera/rgb/image_raw

# View joint states
rostopic echo /joint_states | grep -A 2 head

# List all topics
rostopic list

🎮 MANUAL CONTROL
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# Adjust head tilt (range: -2.09 to 2.09 rad)
rostopic pub /head_tilt_controller/command std_msgs/Float64 "data: 0.5"

# Adjust head pan (range: -2.09 to 2.09 rad)
rostopic pub /head_pan_controller/command std_msgs/Float64 "data: 0.0"

# Check controller status
rosservice call /controller_manager/list_controllers

🔧 TROUBLESHOOTING
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Problem: "No line detected!" warnings
Solution:
  1. Check camera view: rosrun image_view image_view image:=/camera/rgb/image_raw
  2. Verify ground texture is loaded in Gazebo
  3. Adjust HSV thresholds in config/line_follower_params.yaml

Problem: Camera looking wrong direction
Solution:
  1. Rebuild: catkin build
  2. Relaunch simulation
  3. Check head_tilt: rostopic echo /joint_states | grep -A 2 head_tilt

Problem: Debug image not showing in RViz
Solution:
  1. Verify topic: rostopic list | grep debug_image
  2. Check subscribers: rostopic info /line_follower_node/debug_image
  3. Add Image display in RViz using correct topic

📁 CONFIGURATION FILES
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Line detection params:
  config/line_follower_params.yaml

Camera orientation:
  ainex_simulations/ainex_description/urdf/gazebo.xacro (line 41)

Head initialization:
  scripts/line_follower_node.py (lines 167-168)

📊 KEY PARAMETERS
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Camera Pitch:       0.523599 rad (30° downward)
Head Tilt:          0.5 rad (28.6° downward)
Head Pan:           0.0 rad (centered)
Image Size:         640x480 → 160x120 (processing)
HSV Threshold:      V < 50 (black detection)
Control Rate:       10 Hz

✅ VERIFICATION SCRIPT
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Run automated checks:
  ./check_fixes.sh

📖 DOCUMENTATION
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
README.md          - Complete package documentation
USAGE.md           - Quick start guide
FIXES.md           - Detailed fix documentation
FIX_SUMMARY.md     - Summary of applied fixes
SUMMARY.md         - Package overview

🚀 TYPICAL WORKFLOW
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
1. Terminal 1: Launch simulation
   roslaunch ainex_line_follower line_follower_simulation.launch

2. Terminal 2: View camera feed
   rosrun image_view image_view image:=/line_follower_node/debug_image

3. Terminal 3: Monitor detection
   rostopic echo /line_follower_node/line_detected

4. Watch robot follow the line in Gazebo!

💡 TIPS
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
• Ground texture is defined in: gazebo_world/models/line_path_ground_plane/
• Available textures: oval.png, line_path.png, path_1.png, path_2.png
• Change texture in: line_path_ground_plane.material
• Head tilt > 0 = looking down, < 0 = looking up
• Camera pitch > 0 = angled down, < 0 = angled up
• For best line detection, ensure good contrast between line and background

EOF
