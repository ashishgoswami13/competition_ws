# How to Change Robot Spawn Position and Orientation

## ✅ Fix Applied: Camera Topic Corrected

**Issue**: The line follower was subscribing to `/camera/rgb/image_raw` but the camera publishes to `/camera/image_raw`

**Solution**: Updated all launch files and the Python script default to use `/camera/image_raw`

---

## 🤖 Changing Robot Spawn Position and Orientation

### Method 1: Using Launch Arguments (Recommended)

The launch file now supports position and orientation arguments:

```bash
roslaunch ainex_line_follower line_follower_simulation.launch \
    x:=1.0 \
    y:=2.0 \
    z:=0.24 \
    yaw:=1.57
```

### Available Arguments:

| Argument | Description | Default | Units |
|----------|-------------|---------|-------|
| `x` | Position along X-axis | 0 | meters |
| `y` | Position along Y-axis | 0 | meters |
| `z` | Height above ground | 0.24 | meters |
| `roll` | Rotation around X-axis | 0 | radians |
| `pitch` | Rotation around Y-axis | 0 | radians |
| `yaw` | Rotation around Z-axis (heading) | 0 | radians |

### Common Orientation Values:

```bash
# Face North (0°)
yaw:=0

# Face East (90°)
yaw:=1.5708  # π/2 radians

# Face South (180°)
yaw:=3.1416  # π radians

# Face West (270°)
yaw:=4.7124  # 3π/2 radians

# Face Northeast (45°)
yaw:=0.7854  # π/4 radians

# Face Southeast (135°)
yaw:=2.3562  # 3π/4 radians
```

---

## 📋 Examples

### Example 1: Spawn at Origin Facing East
```bash
roslaunch ainex_line_follower line_follower_simulation.launch \
    x:=0 y:=0 yaw:=1.5708
```

### Example 2: Spawn 2 Meters Forward
```bash
roslaunch ainex_line_follower line_follower_simulation.launch \
    x:=2.0 y:=0
```

### Example 3: Spawn at Custom Position, Facing North
```bash
roslaunch ainex_line_follower line_follower_simulation.launch \
    x:=1.5 y:=-0.5 z:=0.24 yaw:=0
```

### Example 4: Spawn Facing the Line Path
```bash
# If your line starts going east, spawn facing east
roslaunch ainex_line_follower line_follower_simulation.launch \
    x:=0 y:=0 yaw:=1.5708
```

### Example 5: Multiple Arguments at Once
```bash
roslaunch ainex_line_follower line_follower_simulation.launch \
    x:=3.0 \
    y:=1.5 \
    z:=0.25 \
    yaw:=0.785 \
    gui:=true \
    paused:=false
```

---

## 🎯 Finding the Right Position

### Step 1: Launch Gazebo in Paused Mode
```bash
roslaunch ainex_line_follower line_follower_simulation.launch paused:=true
```

### Step 2: Use Gazebo GUI to Position Robot
1. In Gazebo, click on the robot
2. Look at the **Pose** section in the left panel
3. Note down the X, Y, Z coordinates and orientation
4. Use those values in your launch command

### Step 3: Relaunch with Correct Position
```bash
roslaunch ainex_line_follower line_follower_simulation.launch \
    x:=<your_x> y:=<your_y> yaw:=<your_yaw>
```

---

## 🔧 Method 2: Edit Launch File Directly

Edit: `src/ainex_line_follower/launch/line_follower_simulation.launch`

Find these lines:
```xml
<!-- Robot spawn position and orientation -->
<arg name="x"               default="0"/>
<arg name="y"               default="0"/>
<arg name="z"               default="0.24"/>
<arg name="roll"            default="0"/>
<arg name="pitch"           default="0"/>
<arg name="yaw"             default="0"/>
```

Change the `default` values:
```xml
<!-- Robot spawn position and orientation -->
<arg name="x"               default="1.5"/>      <!-- Changed -->
<arg name="y"               default="-0.5"/>     <!-- Changed -->
<arg name="z"               default="0.24"/>
<arg name="roll"            default="0"/>
<arg name="pitch"           default="0"/>
<arg name="yaw"             default="1.5708"/>   <!-- Changed: faces East -->
```

Then launch normally:
```bash
roslaunch ainex_line_follower line_follower_simulation.launch
```

---

## 📐 Understanding Coordinates

### Gazebo Coordinate System:
- **X-axis**: Forward (red arrow in Gazebo)
- **Y-axis**: Left (green arrow in Gazebo)
- **Z-axis**: Up (blue arrow in Gazebo)

### Yaw Angles (Rotation around Z-axis):
- **0 rad (0°)**: Robot faces +X direction (East)
- **π/2 rad (90°)**: Robot faces +Y direction (North)
- **π rad (180°)**: Robot faces -X direction (West)
- **3π/2 rad (270°)**: Robot faces -Y direction (South)

### Visual Reference:
```
        +Y (North)
          ↑
          |
          |
-X -------+------- +X (East)
(West)    |
          |
          ↓
        -Y (South)
```

---

## 🚀 Quick Conversion: Degrees to Radians

Use this formula: `radians = degrees × π / 180`

| Degrees | Radians | Direction |
|---------|---------|-----------|
| 0° | 0 | East |
| 45° | 0.7854 | Northeast |
| 90° | 1.5708 | North |
| 135° | 2.3562 | Northwest |
| 180° | 3.1416 | West |
| 225° | 3.9270 | Southwest |
| 270° | 4.7124 | South |
| 315° | 5.4978 | Southeast |

### Python Quick Converter:
```python
import math
degrees = 90
radians = degrees * math.pi / 180
print(f"{degrees}° = {radians:.4f} radians")
```

---

## 🔍 Debugging Spawn Issues

### Check Robot Position After Spawn:
```bash
# Terminal 1: Launch simulation
roslaunch ainex_line_follower line_follower_simulation.launch x:=1.0 yaw:=1.57

# Terminal 2: Check actual position
rostopic echo /gazebo/model_states -n 1
```

### Verify Robot is on Ground:
- Default Z should be `0.24` meters (robot standing height)
- Too low: Robot will be inside ground
- Too high: Robot will fall

### Check Orientation:
```bash
# View current orientation
rostopic echo /gazebo/link_states -n 1 | grep -A 10 "ainex"
```

---

## 💡 Tips for Line Following

### Tip 1: Align with Line Start
Position the robot so it starts on the line:
```bash
roslaunch ainex_line_follower line_follower_simulation.launch \
    x:=0 y:=0 yaw:=1.5708
```

### Tip 2: Give Some Distance
Start slightly before the line to allow camera to detect it:
```bash
roslaunch ainex_line_follower line_follower_simulation.launch \
    x:=-0.5 y:=0 yaw:=1.5708
```

### Tip 3: Spawn at Different Points on Path
Test from different positions:
```bash
# Start position
roslaunch ainex_line_follower line_follower_simulation.launch x:=0 y:=0 yaw:=1.57

# Middle of path
roslaunch ainex_line_follower line_follower_simulation.launch x:=2.0 y:=1.0 yaw:=0.78

# End of path
roslaunch ainex_line_follower line_follower_simulation.launch x:=4.0 y:=2.0 yaw:=0
```

---

## ⚙️ Additional Launch Arguments

You can combine spawn position with other arguments:

```bash
roslaunch ainex_line_follower line_follower_simulation.launch \
    x:=1.0 \
    y:=0.5 \
    yaw:=1.57 \
    gui:=true \          # Show Gazebo GUI
    paused:=true \       # Start paused
    verbose:=true \      # Verbose output
    headless:=false      # Not headless mode
```

---

## 📝 Summary of Changes Made

### Files Modified:
1. ✅ `launch/line_follower_simulation.launch` 
   - Changed image topic from `/camera/rgb/image_raw` to `/camera/image_raw`
   - Added `roll`, `pitch`, `yaw` arguments for orientation
   - Added `-R -P -Y` flags to spawn_model command

2. ✅ `launch/line_follower_only.launch`
   - Changed default image topic to `/camera/image_raw`

3. ✅ `launch/test_line_follower.launch`
   - Changed image topic to `/camera/image_raw`

4. ✅ `scripts/line_follower_node.py`
   - Changed default image topic to `/camera/image_raw`

---

## ✅ Testing the Fixes

### Test 1: Verify Debug Image Works
```bash
# Terminal 1: Launch simulation
roslaunch ainex_line_follower line_follower_simulation.launch

# Terminal 2: View debug image
rosrun image_view image_view image:=/line_follower_node/debug_image
```

You should now see the debug image with line detection!

### Test 2: Verify Custom Spawn Position
```bash
roslaunch ainex_line_follower line_follower_simulation.launch \
    x:=1.0 y:=0.5 yaw:=1.57
```

Check in Gazebo that robot is at (1.0, 0.5) facing north.

### Test 3: Check Line Detection
```bash
# Should show "true" when line is visible
rostopic echo /line_follower_node/line_detected
```

---

## 🎉 You're All Set!

The debug image should now work, and you can easily change the robot's spawn position and orientation using launch arguments!
