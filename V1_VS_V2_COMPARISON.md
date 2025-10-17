# Walking Logic Comparison: V1 vs V2

## Quick Summary

| Feature | V1 (Original) | V2 (Rewritten) |
|---------|---------------|----------------|
| **Code Lines** | 244 | 401 (better structured) |
| **API Used** | `move()` | `set_step()` |
| **Turn Logic** | Simple proportional | Dead zone + proportional + saturation |
| **Speed Control** | Fixed | Adaptive (slows on turns) |
| **State Management** | Flags | Proper state machine |
| **Error Handling** | Basic | Robust with grace periods |

---

## Key Algorithmic Differences

### Turn Angle Calculation

**V1 - Simple Proportional:**
```python
error = (line_x - center_x) / center_x
turn_angle = error * turn_gain * max_turn_angle
turn_angle = clamp(turn_angle, -max_angle, max_angle)
```

**V2 - Segmented Control:**
```python
error = line_x - center_x

if abs(error) < 10:           # Dead zone
    return 0
elif abs(error) < width/6:    # Proportional
    ratio = abs(error) / (width/6)
    angle = min_angle + ratio * (max_angle - min_angle)
    return copysign(angle, error)
else:                         # Saturated
    return copysign(max_angle, error)
```

**Benefit**: Less oscillation, smoother centering, predictable behavior

---

### Forward Speed

**V1:**
```python
# Always uses same forward speed
forward_speed = 0.008m (from config)

# Only exception: when line lost
if line_lost:
    forward_speed = 0.008 * 0.3 = 0.0024m
```

**V2:**
```python
# Dynamic speed based on turn angle
if abs(turn_angle) < 4°:
    speed = 0.010m          # Straight - full speed
elif abs(turn_angle) < 8°:
    speed = 0.006-0.010m    # Turning - interpolated
else:
    speed = 0.006m          # Sharp turn - slow

# Line lost
if line_lost:
    speed = 0.006 * 0.5 = 0.003m
```

**Benefit**: Better balance between speed and stability

---

### Line Loss Handling

**V1:**
```python
if not line_detected:
    if last_line_x is not None:
        # Keep moving forward slowly
        move(speed=4, forward_speed * 0.3, ...)
    else:
        # Stop immediately
        gait_manager.stop()
```

**V2:**
```python
if not line_detected:
    line_lost_count += 1
    
    if line_lost_count < 20 and last_line_x is not None:
        # Grace period - keep searching
        execute_walk_command(speed * 0.5, 0, False)
    else:
        # Lost for too long - stop
        gait_manager.stop()
```

**Benefit**: More forgiving, handles temporary occlusions

---

### Initialization

**V1:**
```python
# Wait for simulation
rospy.sleep(2.0)

# Update pose
gait_manager.update_pose(walking_param)
rospy.sleep(1.0)

# First move flag
if first_move:
    move(speed=4, x=0.003, ...)
    first_move = False
    rospy.sleep(1.5)
    return
```

**V2:**
```python
# Separate initialization method
def initialize_robot():
    rospy.sleep(2.0)
    gait_manager.update_pose(walking_param)
    rospy.sleep(1.5)
    is_initialized = True

# First step in control loop
if not first_step_done:
    execute_walk_command(0.003, 0, False)
    first_step_done = True
    rospy.sleep(1.0)
    return
```

**Benefit**: Clearer separation, better state tracking

---

### Gait Control API

**V1 - Using move():**
```python
self.gait_manager.move(
    speed,           # 1-4 (4=slowest, 600ms)
    x_amplitude,     # meters
    y_amplitude,     # meters
    rotation_angle,  # degrees
    arm_swap=0,
    step_num=0
)
```

**V2 - Using set_step():**
```python
self.gait_manager.set_step(
    dsp,                # [period_ms, dsp_ratio, y_swap]
    x_amplitude,        # meters
    y_amplitude,        # meters
    rotation_angle,     # degrees
    walking_param,      # full gait parameters
    arm_swap=0,
    step_num=0
)
```

**Benefit**: Direct DSP control, matches Reference implementation

---

## Performance Expectations

### V1 Expected Issues:
- ❌ May oscillate around line center due to no dead zone
- ❌ Fixed speed may be too fast for sharp turns
- ❌ Immediate stop on line loss may be too aggressive
- ⚠️ Simple proportional control can overshoot

### V2 Expected Improvements:
- ✅ Dead zone reduces oscillation
- ✅ Speed reduction on turns improves stability  
- ✅ Grace period handles temporary line loss
- ✅ Segmented control is more predictable
- ✅ Better logging for debugging

---

## Parameters Comparison

| Parameter | V1 Value | V2 Value | Notes |
|-----------|----------|----------|-------|
| `walking_speed` | 4 (600ms) | N/A | V2 uses DSP directly |
| `forward_speed` | 0.008m | 0.010m | V2 has higher max |
| `min_forward_speed` | N/A | 0.006m | V2 only - for turns |
| `turn_gain` | 0.6 | N/A | V2 uses different logic |
| `max_turn_angle` | 8.0° | 8.0° | Same |
| `min_turn_angle` | N/A | 4.0° | V2 only - turning threshold |
| `center_tolerance` | N/A | 10px | V2 only - dead zone |
| `step_height` | 0.010m | 0.010m | Same |
| `body_height` | 0.025m | 0.025m | Same |

---

## Code Quality Metrics

### V1:
- Functions: 5 methods
- Comments: Minimal
- Error handling: Basic try-catch
- Logging: Basic with emojis
- State variables: 3 flags

### V2:
- Functions: 8 methods (better separation)
- Comments: Comprehensive docstrings
- Error handling: Graceful degradation
- Logging: Detailed with context
- State variables: Proper state machine

---

## Testing Checklist

**Both versions should:**
- [ ] Initialize without falling
- [ ] Follow straight line smoothly
- [ ] Handle gentle curves
- [ ] Recover from line loss

**V2 specific:**
- [ ] Show dead zone behavior (no oscillation when centered)
- [ ] Slow down appropriately on sharp turns
- [ ] Continue searching for 2 seconds when line lost
- [ ] Display detailed state in logs

---

## Recommendation

**Use V2** because:
1. More robust turn control (dead zone + segmented)
2. Adaptive speed for better stability
3. Better line loss handling
4. Cleaner, more maintainable code
5. Matches Reference implementation patterns
6. Better logging for debugging

**Rollback to V1** if:
- V2 has unexpected bugs
- Need simpler logic for debugging
- V2 performance is worse in practice

---

**Current Status**: V2 is now active in launch file  
**Test Command**: `roslaunch ainex_line_follower line_follower_simulation.launch`
