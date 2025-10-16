# Ainex Line Follower - Quick Start Guide

## Installation & Build

1. **Build the package**:
```bash
cd ~/competition_ws
catkin build ainex_line_follower
# or use: catkin_make
```

2. **Source the workspace**:
```bash
source ~/competition_ws/devel/setup.bash
```

Or run the automated setup script:
```bash
cd ~/competition_ws/src/ainex_line_follower
./setup.sh
```

## Running the Simulation

### Option 1: Full Simulation (Recommended)
This launches everything you need in one command:

```bash
roslaunch ainex_line_follower line_follower_simulation.launch
```

What this does:
- Starts Gazebo with the line follower world
- Spawns the Ainex humanoid robot
- Loads all joint controllers
- Starts the line following algorithm

### Option 2: Separate Components
For debugging or testing, you can run components separately:

**Terminal 1** - Start Gazebo and spawn robot:
```bash
roslaunch gazebo_world spawn_robot_line_follower.launch
```

**Terminal 2** - Start line follower:
```bash
roslaunch ainex_line_follower line_follower_only.launch
```

## Viewing Debug Information

### View Processed Images
To see what the robot sees and how it detects the line:

```bash
rosrun image_view image_view image:=/line_follower/debug_image
```

This shows:
- Green boxes: ROI regions where it looks for the line
- Red contours: Detected line segments
- Blue circle: Center point of the detected line
- Yellow line: Image center reference

### Monitor Topics

Check if line is detected:
```bash
rostopic echo /line_follower/line_detected
```

View camera image:
```bash
rosrun image_view image_view image:=/camera/rgb/image_raw
```

Check robot joint states:
```bash
rostopic echo /joint_states
```

List all active topics:
```bash
rostopic list
```

## Testing Without Gazebo

For algorithm development and testing without the full simulation:

**Terminal 1** - Start test camera (publishes fake images with a moving line):
```bash
rosrun ainex_line_follower simple_camera_publisher.py
```

**Terminal 2** - Start line follower:
```bash
roslaunch ainex_line_follower line_follower_only.launch
```

**Terminal 3** - View debug output:
```bash
rosrun image_view image_view image:=/line_follower/debug_image
```

## Adjusting Parameters

Edit the configuration file to tune performance:

```bash
nano ~/competition_ws/src/ainex_line_follower/config/line_follower_params.yaml
```

Key parameters to adjust:

- **Detection sensitivity**: Adjust `lower_black` and `upper_black` HSV values
- **Turning aggressiveness**: Change `yaw_range` (default: [-8, 8] degrees)
- **Walking speed**: Modify `x_max` (default: 0.010 meters)
- **ROI position**: Adjust `line_roi` to change where it looks for the line

After changes, restart the line follower node.

## Common Issues & Solutions

### Issue: Robot doesn't move
**Solution**: 
- Check that controllers are loaded: `rosservice call /controller_manager/list_controllers`
- Verify Gazebo is running: `ps aux | grep gazebo`
- Make sure robot spawned correctly (you should see it in Gazebo GUI)

### Issue: Line not detected
**Solution**:
- View debug image to see what camera sees
- Adjust HSV thresholds in config file (black line may appear differently in lighting)
- Ensure the line follower world has a visible black line
- Check camera is working: `rostopic hz /camera/rgb/image_raw`

### Issue: No camera images
**Solution**:
- Verify robot URDF includes camera sensor
- Check Gazebo camera plugin is loaded
- Try: `rosrun image_view image_view image:=/camera/rgb/image_raw`

### Issue: Build errors
**Solution**:
- Install missing dependencies: `rosdep install --from-paths src --ignore-src -r -y`
- Make sure cv_bridge is installed: `sudo apt-get install ros-noetic-cv-bridge`
- Clean and rebuild: `catkin clean -y && catkin build`

## Stopping the Simulation

Press `Ctrl+C` in the terminal where you launched the simulation.

To kill all ROS nodes:
```bash
rosnode kill -a
```

To also stop Gazebo:
```bash
pkill -9 gazebo
pkill -9 gzclient
pkill -9 gzserver
```

## Next Steps

1. **Tune parameters** for better performance
2. **Add camera sensor** to robot URDF if not present
3. **Integrate full walking module** from ainex_kinematics for realistic walking
4. **Implement advanced behaviors** like obstacle avoidance or intersection handling

## Useful ROS Commands

```bash
# See all running nodes
rosnode list

# Get info about line follower node
rosnode info /line_follower

# See node computation graph
rqt_graph

# Monitor system performance
rqt_top

# Record data for analysis
rosbag record -a

# View TF tree
rosrun rqt_tf_tree rqt_tf_tree
```

## Package Structure Reference

```
ainex_line_follower/
├── CMakeLists.txt              # Build configuration
├── package.xml                 # Package metadata
├── setup.py                    # Python package setup
├── setup.sh                    # Quick setup script
├── README.md                   # Full documentation
├── USAGE.md                    # This file
├── config/
│   └── line_follower_params.yaml    # Tunable parameters
├── launch/
│   ├── line_follower_simulation.launch   # Full simulation
│   └── line_follower_only.launch         # Node only
├── scripts/
│   ├── line_follower_node.py            # Main controller
│   └── simple_camera_publisher.py       # Test camera
└── src/
    └── ainex_line_follower/
        └── __init__.py
```

## Support

For issues or questions:
1. Check the README.md for detailed documentation
2. Review ROS logs: `~/.ros/log/latest/`
3. Enable verbose output: Add `output="screen"` to launch file nodes

Happy line following! 🤖
