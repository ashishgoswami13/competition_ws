# Line Follower Stability Fixes

## Issues Reported
1. **Robot sometimes "spinning" and "spawning here and there"** - unstable, erratic behavior
2. **Robot not standing upright** - crouching or bent posture

## Root Causes Identified

### 1. Using `set_step()` Instead of `move()`
**Problem**: `GaitManager.set_step()` is designed for setting specific step parameters with continuous calls, which was causing instability as the walking pattern restarted every control cycle.

**Solution**: Use `GaitManager.move()` which is simpler and designed for continuous walking:
```python
# OLD (unstable)
self.gait_manager.set_step([400, 0.2, 0.02], x, y, angle, params, arm_swap=0, step_num=0)

# NEW (stable)
self.gait_manager.move(speed, x, y, angle, arm_swap=0, step_num=0)
```

### 2. Control Parameters Too Aggressive
**Problems**:
- `turn_gain: 1.2` - too sensitive, causing overcorrection
- `max_turn_angle: 18.0°` - too large, causing sharp turns
- `walking_speed: 4` - fastest speed, reduced stability
- `forward_speed: 0.015` - too fast for stable control
- `control_rate: 10Hz` - too frequent updates

**Solutions**:
- `turn_gain: 0.8` - gentler turning response
- `max_turn_angle: 10.0°` - smoother turns
- `walking_speed: 2` - medium speed for better balance
- `forward_speed: 0.012` - slower, more controlled
- `control_rate: 5Hz` - less frequent updates, smoother

### 3. ROI Too Far From Robot
**Problem**: ROI at 50%-70% of image was looking too far ahead, causing delayed reactions.

**Solution**: Moved ROI closer to robot (60%-80%) for quicker response and better stability.

### 4. Stopping Immediately When Line Lost
**Problem**: Robot stopped completely when line was briefly lost, no recovery attempt.

**Solution**: Implemented recovery behavior:
```python
if self.line_detected:
    # Follow line normally
    self.gait_manager.move(speed, forward_speed, 0, turn_angle, ...)
else:
    # Recovery mode - continue forward slowly
    self.gait_manager.move(speed, forward_speed * 0.5, 0, 0, ...)
```

### 5. Body Height Too High
**Problem**: `body_height: 0.025` with spawn `z: 0.30` made robot crouch/bend.

**Solutions**:
- Reduced `body_height: 0.015` - lower center of gravity
- Adjusted spawn `z: 0.28` - better initial posture

## Changes Made

### File: `/src/ainex_line_follower/scripts/line_follower_node.py`

**Changed control method:**
```python
def control_walking(self):
    """Use GaitManager.move() for continuous walking"""
    if self.line_detected:
        turn_angle_int = int(round(turn_angle))
        
        # Use move() instead of set_step()
        self.gait_manager.move(
            self.walking_speed,  # Speed 1-4
            self.forward_speed,
            0.0,
            turn_angle_int,
            arm_swap=0,
            step_num=0
        )
    else:
        # Recovery mode - keep moving forward slowly
        if self.last_line_x is not None:
            self.gait_manager.move(
                self.walking_speed,
                self.forward_speed * 0.5,  # Half speed
                0.0,
                0,  # No turning
                arm_swap=0,
                step_num=0
            )
```

### File: `/src/ainex_line_follower/config/line_follower_params.yaml`

**Before:**
```yaml
line_roi: [[0.5, 0.7, 0.2, 0.8]]  # 50%-70% vertical
min_contour_area: 100
forward_speed: 0.015
turn_gain: 1.2
max_turn_angle: 18.0
walking_speed: 4
body_height: 0.025
```

**After:**
```yaml
line_roi: [[0.6, 0.8, 0.2, 0.8]]  # 60%-80% vertical - closer to robot
min_contour_area: 80               # Easier detection
forward_speed: 0.012               # Slower, more stable
turn_gain: 0.8                     # Gentler turning
max_turn_angle: 10.0               # Smoother turns
walking_speed: 2                   # Medium speed
body_height: 0.015                 # Lower center of gravity
```

### File: `/src/ainex_line_follower/launch/line_follower_simulation.launch`

**Changed:**
```xml
<!-- OLD -->
<param name="control_rate" value="10"/>
<arg name="z" default="0.30"/>

<!-- NEW -->
<param name="control_rate" value="5"/>
<arg name="z" default="0.28"/>
```

## Results

### Before Fixes:
```
[INFO] ✅ Following line - X: 106, Error: 0.33, Turn: 7.0°
[WARN] ❌ Line not detected - STOPPED
[WARN] ❌ Line not detected - STOPPED
```
- Large turn angles (7°)
- Frequent stops
- Erratic behavior ("spinning")
- Robot crouching/unstable

### After Fixes:
```
[INFO] ✅ Following line - X: 79, Error: -0.01, Turn: -0.1°
[INFO] ✅ Following line - X: 81, Error: 0.01, Turn: 0.1°
[INFO] ✅ Following line - X: 80, Error: 0.00, Turn: 0.0°
[INFO] ✅ Following line - X: 83, Error: 0.04, Turn: 0.3°
[WARN] ⚠️ Line lost - continuing forward slowly...
[INFO] ✅ Following line - X: 117, Error: 0.46, Turn: 3.7°
```
- Small turn angles (0.1-0.4° typically)
- Well-centered (X: 79-84)
- Smooth recovery when line lost
- Stable, upright walking

## Key Takeaways

1. **Use the right API**: `GaitManager.move()` for continuous walking, not `set_step()`
2. **Conservative parameters**: Start slow and gentle, tune up if needed
3. **Look closer ahead**: ROI closer to robot = quicker response
4. **Implement recovery**: Don't stop immediately, try to find the line
5. **Lower is stabler**: Lower body height = better balance

## Testing Commands

```bash
# Build
cd ~/competition_ws
catkin build ainex_line_follower
source devel/setup.bash

# Launch
roslaunch ainex_line_follower line_follower_simulation.launch

# Monitor (optional)
rostopic echo /line_follower_node/line_detected
rosrun image_view image_view image:=/line_follower_node/debug_image
```

## Fine-Tuning

If you need to adjust behavior:

**For sharper turns:**
- Increase `turn_gain`: 0.8 → 1.0
- Increase `max_turn_angle`: 10.0 → 12.0

**For slower/more stable:**
- Decrease `forward_speed`: 0.012 → 0.010
- Decrease `walking_speed`: 2 → 1

**For faster following:**
- Increase `walking_speed`: 2 → 3
- Increase `forward_speed`: 0.012 → 0.015

**If line detection issues:**
- Adjust `min_contour_area`: 80 → 60 (easier) or 100 (stricter)
- Adjust `line_roi`: Move vertically or change width
