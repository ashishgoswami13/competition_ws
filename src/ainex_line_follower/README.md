# Ainex Line Follower

Autonomous line following package for the Ainex humanoid robot in Gazebo simulation with obstacle color detection.

## Overview

This package enables the Ainex robot to autonomously navigate a black line path while detecting and identifying colored obstacles (red stairs and blue hurdles) in a simulated Gazebo environment. It uses camera vision, OpenCV for image processing, and the ainex_kinematics GaitManager for stable bipedal walking.

## Features

- ✅ **Black Line Following**: HSV-based color detection with adaptive turning control
- ✅ **Obstacle Color Detection**: Identifies red and blue obstacles in real-time
- ✅ **Common ROI Processing**: Unified region of interest for line and obstacle detection
- ✅ **Visual Feedback**: Live image display with bounding boxes and color labels
- ✅ **Continuous Walking**: Robot maintains last command when line temporarily lost
- ✅ **GaitManager Integration**: Uses Lesson 4 tutorial walking parameters for simulation
- ✅ **Auto-Launch Viewer**: Image visualization starts automatically with simulation

## Quick Start

Launch the complete simulation:

```bash
roslaunch ainex_line_follower line_follower_simulation.launch
```

This will automatically:
1. Start Gazebo with the line follower world (including stairs and hurdles)
2. Spawn the Ainex robot at the correct position and orientation
3. Load position controllers and ainex_kinematics
4. Start the line follower node with obstacle detection
5. Open the image viewer showing detection visualization

## Package Structure

```
ainex_line_follower/
├── config/
│   └── line_follower_params.yaml         # Detection and control parameters
├── launch/
│   └── line_follower_simulation.launch   # Main simulation launch file
├── scripts/
│   ├── line_follower_node_v2.py          # Line follower with obstacle detection
│   └── delayed_start.sh                  # Gazebo unpause script
└── src/
    └── ainex_line_follower/              # Python package
```

## Dependencies

- **ROS Melodic** (Ubuntu 18.04)
- **Gazebo** (simulation environment)
- **OpenCV** (cv2, cv_bridge)
- **ainex_kinematics** (GaitManager for walking control)
- **ainex_description** (robot URDF with camera)
- **ainex_gazebo** (position controllers)
- **gazebo_world** (line follower world with obstacles)

## Configuration

### Walking Parameters (from Lesson 4 Tutorial)

Located in `ainex_kinematics/config/walking_param_sim.yaml`:

```yaml
period_time: 1500         # Walking cycle period (ms)
dsp_ratio: 0.3           # Double support phase ratio
step_fb_ratio: 0.28      # Forward/backward step ratio
y_swap_amplitude: 0.035  # Lateral sway (meters)
pelvis_offset: 0.045     # Pelvis height offset
```

### Detection Parameters

Located in `config/line_follower_params.yaml`:

```yaml
# Image processing
image_process_size: [160, 120]

# Common ROI for line and obstacle detection
common_roi: [0.3, 0.8, 0.1, 0.9]  # [y_start, y_end, x_start, x_end]

# Color detection (HSV)
lower_black: [0, 0, 0]
upper_black: [180, 255, 50]
lower_red_1: [0, 100, 100]
upper_red_1: [10, 255, 255]
lower_red_2: [160, 100, 100]
upper_red_2: [180, 255, 255]
lower_blue: [100, 100, 100]
upper_blue: [130, 255, 255]

# Walking control
max_turn_angle: 10.0     # Maximum turn angle (degrees)
turn_threshold: 4.0      # Angle to switch from go to turn gait
forward_speed: 0.05      # Forward speed straight (meters)
turn_forward_speed: 0.008 # Forward speed when turning
```

## Topics

### Subscribed
- `/camera/image_raw` (sensor_msgs/Image): Camera feed from Gazebo

### Published
- `/line_follower_node/debug_image` (sensor_msgs/Image): Annotated image with detections
- `/robotis/walking/command` (std_msgs/String): Walking gait commands
- `/robotis/walking/set_params` (op3_walking_module_msgs/WalkingParam): Walking parameters

## How It Works

### 1. Image Processing
- Resizes camera image to 160x120 for faster processing
- Converts from BGR to HSV color space
- Extracts common ROI region: [0.3:0.8, 0.1:0.9] (60% vertical, 80% horizontal)

### 2. Line Detection
- Applies HSV thresholding for black color (V < 50)
- Finds contours and selects the largest
- Calculates center X position and line angle
- Draws white bounding box around detected line

### 3. Obstacle Detection
- Detects red obstacles using two HSV ranges (0-10° and 160-180° hue)
- Detects blue obstacles using single HSV range (100-130° hue)
- Draws colored bounding boxes around each detected obstacle
- Displays "Color: Red", "Color: Blue", or "Color: Red + Blue" text

### 4. Walking Control
- **When line detected**: Calculates turn angle from line position
  - If angle > 4°: Uses `turn_gait` with 0.008m forward speed
  - If angle ≤ 4°: Uses `go_gait` with 0.05m forward speed
- **When line lost**: Continues executing last walking command (no stopping)
- Uses GaitManager for smooth bipedal walking with parameters from walking_param_sim.yaml

### 5. Visual Feedback
- White box: Detected black line
- Red boxes: Detected red obstacles (stairs)
- Blue boxes: Detected blue obstacles (hurdles)
- Text: "Color: Red/Blue/Red + Blue" (font size 0.4, minimal display)

## World Configuration

The `gazebo_world/worlds/line_follower.world` includes:

- **Black line track**: Curved path for line following
- **Red stairs**: Pose `[-1.96935, 2.02094, 0, 0, 0, -0.800000]`
- **Blue hurdle**: Pose `[-1.15692, 1.20177, 0, 0, 0, 0.813093]`

## Robot Spawn Configuration

Launch file spawn parameters:

```xml
<arg name="x" default="0"/>
<arg name="y" default="0"/>
<arg name="z" default="0.25"/>
<arg name="yaw" default="90"/>  <!-- 90 degrees -->
```

Joint initialization:
```
l_sho_roll: -1.403, l_el_yaw: -1.226
r_sho_roll: 1.403, r_el_yaw: 1.226
```

## Algorithm Details

### Reference Implementation Approach
The walking logic is based on the Reference `combination_node.py`:

```python
def control_walking(self, line_data):
    if line_data is None:
        return  # Continue last command, don't stop
    
    # Process line data and send new command
    self.gait_manager.process(line_data.x_move_amplitude,
                              line_data.y_move_amplitude, 
                              line_data.angle)
```

**Key insight**: Robot does NOT stop when line is temporarily lost—it continues with the last successful walking command for smooth navigation.

### Speed Calculation
```python
if abs(angle) > 6.0:  # Sharp turn
    forward_speed = 0.008  # Slow down
else:  # Straight or gentle curve
    forward_speed = 0.05   # Normal speed
```

Robot always moves forward, even when turning sharply, to maintain continuous progress along the path.

## Troubleshooting

### Robot not following line
- Check image viewer: `Color detected? Bounding boxes shown?`
- Verify camera topic: `rostopic hz /camera/image_raw`
- Adjust HSV thresholds in `line_follower_params.yaml`

### Robot oscillating or unstable
- Walking parameters may need tuning for your system
- Check Gazebo physics are not running too fast
- Verify robot spawns correctly (z=0.25, not on ground)

### No image viewer
- Image viewer auto-launches after 8 seconds
- Manual launch: `rosrun rqt_image_view rqt_image_view /line_follower_node/debug_image`

### Colors not detected
- Check lighting in Gazebo world
- Verify obstacle models are loaded correctly
- Adjust HSV ranges for red/blue in config file

## Performance Notes

- **Control Rate**: 10 Hz (100ms cycle time)
- **Image Resolution**: 160x120 (downsampled from camera for speed)
- **Walking Period**: 1500ms (slower than real robot for simulation stability)
- **ROI Coverage**: 60% vertical × 80% horizontal of image

## Credits

- Based on Reference `visual_patrol.py` and `combination_node.py`
- Walking parameters from Lesson 4 Gazebo Simulation Tutorial
- Adapted for Gazebo simulation with obstacle detection features

## License

Apache-2.0
