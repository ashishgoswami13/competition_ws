# Common ROI Implementation for Line Following & Obstacle Detection

## Overview
Updated the line follower to use a **single common ROI** for both line following and obstacle color detection. This allows simultaneous detection of the black line, red stairs, and blue hurdles in the same vision region.

## Key Changes

### 1. **Unified ROI Parameter**
```python
# OLD: Separate ROIs
self.line_roi = [[0.5, 0.7, 0.2, 0.8]]        # Line detection only
self.obstacle_roi = [0.1, 0.5, 0.2, 0.8]      # Obstacle detection only

# NEW: Single common ROI
self.common_roi = [0.3, 0.8, 0.1, 0.9]        # Both line & obstacles
```

**Common ROI format:** `[y_min, y_max, x_min, x_max]` as fractions of image size
- **y_min: 0.3** (30% from top) - Start looking here
- **y_max: 0.8** (80% from top) - End looking here  
- **x_min: 0.1** (10% from left) - Left boundary
- **x_max: 0.9** (90% from right) - Right boundary

This expanded ROI covers:
- **Upper region** for obstacles (stairs, hurdles)
- **Middle region** for line tracking
- **Lower region** for immediate path detection

### 2. **Enhanced Multi-Object Detection**
Both methods now detect **ALL instances** of colored objects:

```python
# OLD: Only detected largest contour
largest_red = max(contours_red, key=cv2.contourArea)

# NEW: Detects all valid contours
for contour in contours_red:
    if contour_area > min_area:
        # Label each instance: STAIRS #1, STAIRS #2, etc.
```

### 3. **Visual Feedback Improvements**

#### Debug Image Display:
- **Green ROI box** (thick, 2px) - Shows common detection region
- **Red bounding boxes** around stairs with "STAIRS #1", "STAIRS #2" labels
- **Blue bounding boxes** around hurdles with "HURDLE #1", "HURDLE #2" labels
- **Summary text** (top-right): Shows count of each obstacle type
  - Example: "Detected: RED x2, BLUE x1"

#### Terminal Output:
When obstacles side-by-side are detected:
```
🚧 OBSTACLE DETECTED: 🔴 STAIRS (RED) + 🔵 HURDLE (BLUE)
```

### 4. **Code Structure**

#### Modified Methods:

**`detect_line()`**
- Uses `self.common_roi` instead of `self.line_roi`
- Draws the common ROI box (green, 2px thick)
- Adds "Common ROI" label
- Simplified from multi-ROI loop to single ROI

**`detect_obstacles()`**
- Uses `self.common_roi` instead of `self.obstacle_roi`
- Does NOT redraw ROI box (already drawn by `detect_line()`)
- Iterates through ALL contours for each color
- Labels each instance with count: #1, #2, etc.
- Shows count summary in top-right corner

## Configuration

### Launch File Parameter (Optional)
Add to your launch file to customize the ROI:

```xml
<!-- Common ROI for line & obstacle detection [y_min, y_max, x_min, x_max] -->
<param name="common_roi" value="[0.3, 0.8, 0.1, 0.9]"/>
```

### Tuning Guidelines:

**To detect obstacles earlier** (farther ahead):
```python
common_roi = [0.2, 0.8, 0.1, 0.9]  # Start higher (20% from top)
```

**To focus on nearby obstacles**:
```python
common_roi = [0.4, 0.8, 0.1, 0.9]  # Start lower (40% from top)
```

**To widen field of view**:
```python
common_roi = [0.3, 0.8, 0.0, 1.0]  # Full width (0% to 100%)
```

**To narrow focus** (reduce false positives):
```python
common_roi = [0.3, 0.8, 0.2, 0.8]  # Center 60% only
```

## Benefits of Common ROI

### ✅ Advantages:
1. **Unified Vision** - Robot looks at one region for all detection tasks
2. **Simultaneous Detection** - Can see line AND obstacles at once
3. **Side-by-side Detection** - Detects multiple obstacles next to each other
4. **Simplified Code** - Single ROI parameter to tune
5. **Better Performance** - Processes same HSV image for all colors
6. **Consistent Framing** - All detections relative to same region

### 📊 Detection Capabilities:
- ✅ Black line detection (for following)
- ✅ Multiple red stairs (labeled #1, #2, ...)
- ✅ Multiple blue hurdles (labeled #1, #2, ...)
- ✅ Simultaneous RED + BLUE detection
- ✅ Count display for each color type

## Testing

### Test Scenario: Side-by-Side Obstacles

When robot sees both stairs (red) and hurdle (blue) together:

**Debug Image Shows:**
```
┌──────── Common ROI (green box) ────────┐
│                                         │
│  [STAIRS #1]  (red box)                │
│  [HURDLE #1]  (blue box)               │
│                                         │
│  ═══════ (black line)                  │
│                                         │
└─────────────────────────────────────────┘
           Top-right: "Detected: RED x1, BLUE x1"
```

**Terminal Shows:**
```
🚧 OBSTACLE DETECTED: 🔴 STAIRS (RED) + 🔵 HURDLE (BLUE)
```

### Test Commands:

```bash
# Terminal 1: Launch simulation
roslaunch ainex_gazebo ainex_gazebo.launch

# Terminal 2: View debug image
rosrun rqt_image_view rqt_image_view /line_follower_node/debug_image
```

### Expected Behavior:

1. **Green ROI box** visible on debug image (labeled "Common ROI")
2. **Red boxes** appear around stairs when visible
3. **Blue boxes** appear around hurdles when visible
4. **Both colors** detected when side-by-side
5. **Count summary** shows: "Detected: RED x[count], BLUE x[count]"
6. **Terminal logs** obstacle detection state changes
7. **Line following** continues while detecting obstacles

## Technical Details

### ROI Coordinate System:
- **Origin (0, 0)**: Top-left corner of image
- **Values**: Normalized fractions (0.0 to 1.0)
- **Default image size**: 160x120 pixels (after resize)

### Example ROI Translation:
```python
common_roi = [0.3, 0.8, 0.1, 0.9]

# On 160x120 image:
y_min = 0.3 * 120 = 36 pixels from top
y_max = 0.8 * 120 = 96 pixels from top
x_min = 0.1 * 160 = 16 pixels from left
x_max = 0.9 * 160 = 144 pixels from left

# ROI size: 128x60 pixels (80% width, 50% height)
```

### Color Detection Process:
1. Resize input image to 160x120
2. Convert to HSV color space (once)
3. Extract common ROI region
4. Apply black mask → detect line contour
5. Apply red mask (2 ranges) → detect stairs contours
6. Apply blue mask → detect hurdle contours
7. Draw all detections on debug image
8. Publish debug image and log to terminal

## Troubleshooting

### Issue: Not detecting obstacles
**Solution:** Expand ROI to include obstacle position
```python
common_roi = [0.2, 0.9, 0.0, 1.0]  # Larger ROI
```

### Issue: Too many false positives
**Solution:** Increase minimum area threshold
```python
self.min_obstacle_area = 200  # Larger threshold
```

### Issue: Missing line detection
**Solution:** Ensure ROI includes lower portion
```python
common_roi = [0.3, 0.8, 0.1, 0.9]  # Should work for line in lower-middle area
```

### Issue: Only detecting one of two side-by-side obstacles
**Solution:** Check if both are in ROI and above min_area
- Widen ROI horizontally: `x_min=0.0, x_max=1.0`
- Lower area threshold: `min_obstacle_area = 50`

## Performance Notes

- **Frame rate**: ~10 Hz (same as line following)
- **ROI processing**: All color detections use same ROI extraction
- **Multi-object support**: No limit on number of instances per color
- **Memory efficient**: Single ROI, single HSV conversion
- **Label overlap prevention**: Labels positioned above each bounding box

## Summary

The common ROI approach provides:
- ✅ **Unified detection region** for line and obstacles
- ✅ **Side-by-side detection** of multiple colored obstacles
- ✅ **Individual labeling** for each obstacle instance
- ✅ **Count tracking** displayed on screen and in terminal
- ✅ **Simplified configuration** with single ROI parameter
- ✅ **Better visual feedback** with color-coded bounding boxes

Perfect for competition scenarios where obstacles may appear together!
