# Color Detection Features Added

## Overview
Added OpenCV color detection to identify stairs (RED) and hurdles (BLUE) in the camera feed.

## Features Implemented

### 1. **HSV Color Range Detection**
- **RED Detection** (Stairs): Two HSV ranges to handle red wrapping around 0/180
  - Range 1: H: 0-10, S: 100-255, V: 100-255
  - Range 2: H: 160-180, S: 100-255, V: 100-255
  
- **BLUE Detection** (Hurdle): Single HSV range
  - H: 100-130, S: 100-255, V: 100-255

### 2. **Obstacle Detection ROI**
- Located in upper part of image (0.1 to 0.5 normalized height)
- X range: 0.2 to 0.8 (centered view)
- Minimum obstacle area: 100 pixels (configurable via parameter)

### 3. **Visual Feedback on Debug Image**
When obstacles are detected, the debug image shows:
- **Cyan ROI box** showing the obstacle detection region
- **Red bounding box** around detected stairs with "STAIRS (RED)" label
- **Blue bounding box** around detected hurdles with "HURDLE (BLUE)" label
- **Summary text** in top-right corner listing all detected obstacles
- White text labels with colored backgrounds for visibility

### 4. **Terminal Logging**
- Logs obstacle detection when first detected: `🚧 OBSTACLE DETECTED: 🔴 STAIRS (RED)`
- Logs when multiple obstacles detected: `🚧 OBSTACLE DETECTED: 🔴 STAIRS (RED) + 🔵 HURDLE (BLUE)`
- Logs when obstacles clear: `✅ No obstacles detected`
- Only logs when detection state changes to avoid spam

## Configuration Parameters

Add these to your launch file if you want to customize:

```xml
<!-- Obstacle detection ROI [y_min, y_max, x_min, x_max] as fractions of image size -->
<param name="obstacle_roi" value="[0.1, 0.5, 0.2, 0.8]"/>

<!-- Minimum contour area to consider as obstacle (pixels) -->
<param name="min_obstacle_area" value="100"/>
```

## How to View

1. **Debug Image Topic**: `/line_follower_node/debug_image`
   - Shows colored bounding boxes and labels
   - Shows ROI regions
   - Shows detected obstacles summary

2. **Terminal Output**: 
   - Run: `roslaunch ainex_gazebo ainex_gazebo.launch`
   - Watch for obstacle detection messages

3. **View Debug Image**:
   ```bash
   rosrun rqt_image_view rqt_image_view /line_follower_node/debug_image
   ```

## Code Changes

### Modified File
- `/home/ubuntu/competition_ws/src/ainex_line_follower/scripts/line_follower_node_v2.py`

### New Methods
- `detect_obstacles(image, debug_image)`: Main color detection logic
  - Detects red and blue objects in obstacle ROI
  - Draws bounding boxes and labels
  - Returns list of detected colors

### New State Variables
- `self.detected_colors`: Current frame's detected colors
- `self.last_obstacle_detection`: Previous detection state (for change detection)
- HSV color range parameters for red and blue

### Modified Methods
- `run()`: Now calls `detect_obstacles()` and logs results to terminal

## Expected Behavior

1. When robot approaches **stairs** (red color):
   - Red bounding box appears on debug image
   - Label "STAIRS (RED)" displayed
   - Terminal shows: `🚧 OBSTACLE DETECTED: 🔴 STAIRS (RED)`

2. When robot approaches **hurdle** (blue color):
   - Blue bounding box appears on debug image
   - Label "HURDLE (BLUE)" displayed
   - Terminal shows: `🚧 OBSTACLE DETECTED: 🔵 HURDLE (BLUE)`

3. When both obstacles visible:
   - Both bounding boxes shown
   - Terminal shows: `🚧 OBSTACLE DETECTED: 🔴 STAIRS (RED) + 🔵 HURDLE (BLUE)`

4. When no obstacles:
   - Only obstacle ROI box visible (cyan)
   - Terminal shows: `✅ No obstacles detected` (when clearing from previous detection)

## Testing

To test the color detection:

```bash
# Terminal 1: Launch Gazebo with the world containing stairs and hurdles
roslaunch ainex_gazebo ainex_gazebo.launch

# Terminal 2: View the debug image
rosrun rqt_image_view rqt_image_view /line_follower_node/debug_image

# Terminal 3: Monitor the terminal output
# (already visible in Terminal 1)
```

The robot will detect and display the colored obstacles as it follows the line!

## Tuning Tips

If detection isn't working well:

1. **Adjust HSV ranges**: Modify `lower_red1/2`, `upper_red1/2`, `lower_blue`, `upper_blue`
2. **Adjust obstacle ROI**: Change `obstacle_roi` parameter to look higher/lower in image
3. **Adjust minimum area**: Change `min_obstacle_area` to filter noise
4. **Check Gazebo lighting**: Ensure good lighting in simulation world

## Notes

- Color detection runs at the same rate as line following (10 Hz by default)
- HSV color space is more robust to lighting changes than RGB
- Red color requires two ranges because it wraps around the hue circle (0°/360°)
- Detection only triggers logging when state changes to reduce terminal spam
