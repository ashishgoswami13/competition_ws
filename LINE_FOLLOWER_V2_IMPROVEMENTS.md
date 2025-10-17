# Line Follower V2 - Improved Walking Logic

## Date: 2025-10-17

## Overview
Complete rewrite of the walking logic for better stability, cleaner code, and more robust control based on the Reference `visual_patrol` implementation.

## Key Improvements

### 1. **Cleaner Architecture**
- Separated concerns into clear methods
- Better state management
- More intuitive parameter names
- Comprehensive documentation

### 2. **Improved Turn Control**
- **Dead Zone**: No turning when line is within ±10 pixels of center
- **Proportional Control**: Gradual turn angle increase based on error
- **Maximum Turn**: Caps at 8° for stability
- **Speed Reduction**: Automatically slows down when turning hard

### 3. **Better Speed Management**
```python
# Speed varies based on turning:
- Straight walking: 0.010m (max speed)
- Moderate turn:    0.006-0.010m (interpolated)
- Sharp turn:       0.006m (min speed)
```

### 4. **Enhanced Line Loss Handling**
- Tracks consecutive frames without line detection
- Continues searching for 20 frames before stopping
- Reduces speed to 50% when searching
- Clear logging of line loss state

### 5. **Smoother Initialization**
- Gradual pose update using `update_pose()`
- Very gentle first step (0.003m)
- 1 second pause after first step
- Clear startup sequence logging

### 6. **Reference-Based Implementation**
Using `set_step()` instead of `move()` for better control:
```python
self.gait_manager.set_step(
    dsp,                    # [period_ms, dsp_ratio, y_swap]
    forward_speed,          # x_amplitude (meters)
    0,                      # y_amplitude (meters)
    int(-turn_angle),       # rotation_angle (degrees)
    self.walking_param,     # gait parameters
    arm_swap=0,             # No arm swing for stability
    step_num=0              # Continuous walking
)
```

## Walking Parameters

### Gait Configuration
```python
walking_param['body_height'] = 0.025m      # Stable height
walking_param['step_height'] = 0.010m      # REDUCED for stability
walking_param['hip_pitch_offset'] = 15
walking_param['z_swap_amplitude'] = 0.005m # REDUCED for stability
```

### DSP (Double Support Phase)
```python
straight_dsp = [600, 0.2, 0.02]  # 600ms period (SLOWEST)
turn_dsp = [600, 0.2, 0.02]      # Same slow speed when turning
```

### Control Parameters
```yaml
center_tolerance: 10          # pixels - dead zone for centering
max_turn_angle: 8.0           # degrees - maximum turn
min_turn_angle: 4.0           # degrees - threshold for "turning"
max_forward_speed: 0.010      # meters - when going straight
min_forward_speed: 0.006      # meters - when turning hard
```

## Turn Angle Calculation Logic

```python
error = line_x - center_x

if abs(error) < 10 pixels:
    # Dead zone - go straight
    turn_angle = 0

elif abs(error) < width/6:
    # Proportional control
    turn_angle = min_angle + ratio * (max_angle - min_angle)

else:
    # Maximum turn
    turn_angle = ±8°
```

## Forward Speed Calculation Logic

```python
if abs(turn_angle) < 4°:
    # Going straight
    forward_speed = 0.010m

elif abs(turn_angle) < 8°:
    # Moderate turn - interpolate
    forward_speed = 0.006m to 0.010m

else:
    # Sharp turn - minimum speed
    forward_speed = 0.006m
```

## Comparison: Old vs New

| Aspect | Old Implementation | New Implementation |
|--------|-------------------|-------------------|
| **API** | `move()` - simpler | `set_step()` - more control |
| **Turn Logic** | Simple proportional | Dead zone + proportional + max |
| **Speed** | Fixed forward speed | Adaptive based on turn angle |
| **Line Loss** | Simple fallback | 20-frame grace period |
| **First Step** | Flag-based | State machine with pause |
| **Code Quality** | Mixed concerns | Clean separation |
| **Logging** | Basic | Detailed with emojis |

## Testing Recommendations

1. **Startup**: Robot should stand smoothly without jerking
2. **First Step**: Very small, gentle movement with 1s pause
3. **Straight Line**: Should walk at 0.010m without turning
4. **Gentle Curves**: Should slow down proportionally
5. **Sharp Turns**: Should reduce to 0.006m speed
6. **Line Loss**: Should search forward for ~2 seconds before stopping
7. **Recovery**: Should resume smoothly when line is found

## Expected Behavior

- ✅ No jerking or falling during initialization
- ✅ Smooth, stable walking at 600ms period
- ✅ Accurate line following with minimal oscillation
- ✅ Appropriate speed reduction during turns
- ✅ Graceful handling of temporary line loss
- ✅ Clear logging of all state changes

## Files Modified

1. **Created**: `line_follower_node_v2.py` (new implementation)
2. **Modified**: `line_follower_simulation.launch` (uses V2 script)
3. **Unchanged**: `line_follower_params.yaml` (parameters still apply)
4. **Backup**: `line_follower_node.py` (original kept for reference)

## Rollback Instructions

If V2 has issues, revert the launch file:
```xml
<node name="line_follower" pkg="ainex_line_follower" type="line_follower_node.py" ...>
```

## Next Steps

1. Test in simulation
2. Fine-tune turn control if needed
3. Adjust speed parameters based on performance
4. Monitor stability over longer runs

---

**Author**: GitHub Copilot  
**Status**: Ready for testing
