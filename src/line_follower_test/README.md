# ainex_line_follower

Line following package for the Ainex humanoid robot in Gazebo simulation. This package enables the robot to autonomously follow a black line path using camera vision.

## Overview

This package is designed for **simulation only** and is based on the real-world line following functionality from the Reference packages. It has been adapted to work in Gazebo without hardware dependencies.

## Features

- **Line Detection**: Uses OpenCV to detect black lines in camera images using HSV color space
- **Region of Interest (ROI)**: Processes three ROI regions for robust line detection
- **Visual Feedback**: Publishes debug images showing detected lines and ROI regions
- **Configurable Parameters**: All detection and control parameters are configurable via YAML

## Package Contents

```
ainex_line_follower/
├── config/
│   └── line_follower_params.yaml    # Configuration parameters
├── launch/
│   ├── line_follower_simulation.launch   # Full simulation with robot spawn
│   └── line_follower_only.launch         # Line follower node only
├── scripts/
│   ├── line_follower_node.py            # Main line following controller
│   └── simple_camera_publisher.py       # Test camera image publisher
└── src/
    └── ainex_line_follower/             # Python package (future expansion)
```

## Dependencies

- ROS Noetic
- OpenCV (cv2)
- cv_bridge
- ainex_description (robot URDF)
- ainex_gazebo (Gazebo controllers)
- gazebo_world (line follower world)

## Installation

1. Clone this package into your catkin workspace:
```bash
cd ~/competition_ws/src
```

2. Build the package:
```bash
cd ~/competition_ws
catkin build ainex_line_follower
# or
catkin_make
```

3. Source the workspace:
```bash
source ~/competition_ws/devel/setup.bash
```

## Usage

### Full Simulation (Recommended)

Launch the complete simulation with robot spawning and line following:

```bash
roslaunch ainex_line_follower line_follower_simulation.launch
```

This will:
1. Start Gazebo with the line follower world
2. Spawn the Ainex robot at the starting position
3. Load the joint controllers
4. Start the line follower node

### Line Follower Node Only

If you already have the robot spawned in Gazebo:

```bash
roslaunch ainex_line_follower line_follower_only.launch
```

### Test with Simulated Camera

For testing without Gazebo (useful for algorithm development):

```bash
# Terminal 1: Start the test camera publisher
rosrun ainex_line_follower simple_camera_publisher.py

# Terminal 2: Start the line follower (will use test images)
roslaunch ainex_line_follower line_follower_only.launch
```

## Configuration

Edit `config/line_follower_params.yaml` to adjust:

- **Image Processing**: Resolution and ROI regions
- **Color Detection**: HSV thresholds for black line
- **Control Parameters**: Turning angles, step sizes
- **Gait Parameters**: Walking speed and stability

### Key Parameters

```yaml
# Image size for processing (smaller = faster)
image_process_size: [160, 120]

# Black line detection (HSV color space)
lower_black: [0, 0, 0]
upper_black: [180, 255, 50]

# Control parameters
yaw_range: [-8, 8]  # Turning angle range (degrees)
x_max: 0.010        # Maximum forward step (meters)
```

## Topics

### Subscribed Topics
- `/camera/rgb/image_raw` (sensor_msgs/Image): Camera image input

### Published Topics
- `~debug_image` (sensor_msgs/Image): Processed image with detection visualization
- `~line_detected` (std_msgs/Bool): Whether a line is currently detected

### Joint Command Topics
The node publishes to individual joint controllers:
- `/l_hip_yaw_controller/command`
- `/r_hip_yaw_controller/command`
- (and other leg joints)

## Parameters

- `~image_topic` (string, default: "/camera/rgb/image_raw"): Camera topic to subscribe
- `~control_rate` (int, default: 10): Control loop frequency in Hz
- `~auto_start` (bool, default: true): Start line following automatically

## How It Works

1. **Image Acquisition**: Subscribes to camera images from Gazebo
2. **Preprocessing**: Resizes and converts image to HSV color space
3. **Line Detection**: 
   - Applies color thresholding to detect black pixels
   - Processes three ROI regions (upper, center, lower)
   - Finds contours and calculates line center position
4. **Control Calculation**: 
   - Compares line position to image center
   - Calculates turning angle (yaw) based on offset
   - Adjusts forward speed based on turning amount
5. **Motion Execution**: Publishes joint commands to walk and turn

## Differences from Real-World Version

This simulation version has been simplified compared to the real-world implementation:

- **No Hardware SDK**: Removed dependencies on `ainex_sdk` and servo control
- **Simplified Walking**: Uses basic joint control instead of full walking module
- **No Motion Manager**: Direct joint commands instead of action sequences
- **No Service Interface**: Simplified start/stop mechanism

## Troubleshooting

### No camera image
- Check that Gazebo is running and the robot has a camera sensor
- Verify the camera topic name: `rostopic list | grep camera`
- Make sure the robot is spawned correctly

### Robot not moving
- Check joint controller status: `rostopic echo /joint_states`
- Verify controllers are loaded: `rosservice call /controller_manager/list_controllers`

### Line not detected
- View debug image: `rosrun image_view image_view image:=/line_follower/debug_image`
- Adjust HSV thresholds in config file
- Check lighting conditions in Gazebo world

## Future Enhancements

- Integration with full walking module from ainex_kinematics
- Support for different line colors
- Path planning for intersections
- Obstacle avoidance
- Performance metrics and logging

## Credits

Based on the visual_patrol package from the Ainex Reference implementation, adapted for Gazebo simulation.

## License

Apache-2.0
