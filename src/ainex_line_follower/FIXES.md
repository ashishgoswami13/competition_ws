# Line Follower Fixes - Camera Orientation & Debug Image

## Issues Fixed

### 1. Robot Camera Looking Upward
**Problem**: The robot's head was pointing straight up instead of looking at the ground.

**Root Cause**: 
- Head tilt and pan controllers were not initialized in the line_follower_node.py
- Camera orientation in gazebo.xacro had extreme angles

**Solutions Applied**:

#### A. Added Head Controllers to Line Follower Node
**File**: `scripts/line_follower_node.py`

```python
# Added head_pan and head_tilt to joint publishers
joints = [
    'l_hip_yaw', 'l_hip_roll', 'l_hip_pitch', 'l_knee', 'l_ank_pitch', 'l_ank_roll',
    'r_hip_yaw', 'r_hip_roll', 'r_hip_pitch', 'r_knee', 'r_ank_pitch', 'r_ank_roll',
    'l_sho_pitch', 'r_sho_pitch',
    'head_pan', 'head_tilt'  # <-- ADDED
]
```

#### B. Set Initial Head Position
**File**: `scripts/line_follower_node.py`

```python
init_positions = {
    # ... other joints ...
    'head_pan': 0.0,           # Center position
    'head_tilt': 0.5,          # Tilt down to look at ground (~28.6 degrees)
}
```

#### C. Fixed Camera Sensor Orientation
**File**: `ainex_simulations/ainex_description/urdf/gazebo.xacro`

**Before**:
```xml
<pose>0 0 0 ${M_PI} -${M_PI/2} -${M_PI/2}</pose>
```
This caused extreme rotations: Roll=180°, Pitch=-90°, Yaw=-90°

**After**:
```xml
<pose>0 0 0 0 0.523599 0</pose>
```
This provides: Roll=0°, Pitch=30° (downward tilt), Yaw=0°

### 2. Debug Image Not Visible in RViz
**Problem**: The debug image topic wasn't showing in RViz.

**Root Cause**: 
- Topic name mismatch (user may have been looking at wrong topic)
- Gazebo must be running for images to be published

**Solution**: The topic is correctly published as `/line_follower_node/debug_image`

## How to Verify the Fixes

### Step 1: Launch the Simulation
```bash
source ~/competition_ws/devel/setup.bash
roslaunch ainex_line_follower line_follower_simulation.launch
```

### Step 2: Check Camera Feed
In a new terminal:
```bash
# View raw camera feed
rosrun image_view image_view image:=/camera/rgb/image_raw

# OR view debug image with line detection overlay
rosrun image_view image_view image:=/line_follower_node/debug_image
```

### Step 3: View in RViz
```bash
# Launch RViz
rosrun rviz rviz
```

In RViz:
1. Click **Add** button
2. Select **By topic**
3. Find `/line_follower_node/debug_image`
4. Add **Image** display
5. You should see the camera view with line detection visualization

### Step 4: Check Head Position
In RViz or Gazebo, verify that:
- The robot's head is tilted downward
- The camera view shows the ground and line path

## Expected Camera View

After the fixes, the camera should show:
- ✅ Ground plane visible
- ✅ Black line in the center/lower portion of the image
- ✅ ROI regions drawn as green rectangles
- ✅ Detected line highlighted in red
- ✅ Blue circle at line center
- ✅ Yellow vertical line at image center

## Debug Image Topic Details

**Topic Name**: `/line_follower_node/debug_image`
**Message Type**: `sensor_msgs/Image`
**Frame ID**: camera_link
**Encoding**: bgr8
**Resolution**: 160x120 (processed size)

## Troubleshooting

### If camera still looks upward:
1. Check that Gazebo is running
2. Verify head_tilt controller is loaded:
   ```bash
   rosservice call /controller_manager/list_controllers
   ```
3. Manually adjust head:
   ```bash
   rostopic pub /head_tilt_controller/command std_msgs/Float64 "data: 0.5"
   ```

### If no line detected:
1. Check camera view with image_view
2. Verify the ground plane has the line texture
3. Adjust HSV thresholds in `config/line_follower_params.yaml`
4. Check robot height - it should be spawned at z=0.24

### If debug image is blank:
1. Ensure Gazebo is running and publishing camera images
2. Check image topic:
   ```bash
   rostopic hz /camera/rgb/image_raw
   ```
3. Verify the line_follower_node is running:
   ```bash
   rosnode list | grep line_follower
   ```

## Technical Details

### Camera Orientation Angles
- **Pitch = 0.523599 rad = 30 degrees**
- This angles the camera downward to see the ground
- Adjust this value in `gazebo.xacro` if needed (range: 0.3 to 0.8 rad)

### Head Tilt Angle
- **head_tilt = 0.5 rad ≈ 28.6 degrees**
- Positive values tilt the head down
- Range: -2.09 to 2.09 rad (±120 degrees)
- Adjust in `line_follower_node.py` init_walking_pose()

### Joint Limits (from URDF)
- head_pan: -2.09 to 2.09 rad (left-right rotation)
- head_tilt: -2.09 to 2.09 rad (up-down tilt)

## Files Modified

1. ✅ `/src/ainex_line_follower/scripts/line_follower_node.py`
   - Added head_pan and head_tilt to joint publishers
   - Set initial head_tilt to 0.5 rad (downward)

2. ✅ `/src/ainex_simulations/ainex_description/urdf/gazebo.xacro`
   - Changed camera pose from extreme angles to 30° downward pitch

## Verification Commands

```bash
# Check if head controllers are loaded
rosservice call /controller_manager/list_controllers

# Check current head position
rostopic echo /joint_states -n 1 | grep -A 2 head

# Monitor line detection
rostopic echo /line_follower_node/line_detected

# Check camera image rate
rostopic hz /camera/rgb/image_raw

# View debug image
rosrun image_view image_view image:=/line_follower_node/debug_image
```

## Next Steps

After verifying the fixes work:
1. Fine-tune head_tilt angle for optimal ground view
2. Adjust camera pitch in gazebo.xacro if needed
3. Calibrate HSV thresholds for your specific lighting
4. Test line following behavior

## Notes

- The camera is mounted on `body_link`, not on `head_tilt_link`
- This means the head joint angles don't affect the camera view
- The camera orientation is set in the Gazebo sensor plugin
- Both fixes work together: head aesthetics + camera functionality
