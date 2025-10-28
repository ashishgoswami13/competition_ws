# Rectangle Walker - Smooth Human-like Motion

## Overview
Updated the rectangle walker to use proven gait parameters from the Reference implementation (`visual_patrol.py`) to achieve smooth, human-like walking motion.

## Key Improvements

### 1. **Faster DSP (Double Support Phase) Timing**
- **Before**: 600ms (walking), 700ms (turning) - TOO SLOW, jerky motion
- **After**: 300ms (walking), 400ms (turning) - from Reference
- **Result**: Smoother, more natural cadence

### 2. **Optimized Gait Parameters**

#### Forward Walking Gait (`go_gait_param`)
```python
body_height = 0.025m          # Natural standing height
step_height = 0.015m          # Moderate step lift (was 0.008m)
hip_pitch_offset = 15°        # Forward lean for balance (NEW)
z_swap_amplitude = 0.006m     # Natural weight transfer (was 0.004m)
arm_swap = 30°                # Natural arm swing (was 0°)
```

#### Turning Gait (`turn_gait_param`)
```python
body_height = 0.025m
step_height = 0.020m          # Slightly higher for stability
hip_pitch_offset = 15°
z_swap_amplitude = 0.006m
arm_swap = 30°
```

### 3. **Improved Step Speeds**
- **Forward speed**: 0.006m → 0.010m per step (66% faster)
- **Turn speed**: 8° → 6° per step (smoother, more controlled)

### 4. **Natural Turn Mechanics**
- Added small forward movement (0.004m) during turns
- More human-like turning behavior
- Better balance during rotation

### 5. **Reduced Inter-step Delays**
- **Before**: 0.05s between steps
- **After**: 0.01s between steps
- **Result**: Continuous, flowing motion

## Reference Source
Based on proven real-robot implementation:
- **File**: `/Reference/ainex_example/src/ainex_example/visual_patrol.py`
- **Used by**: Real Ainex robot for competition tasks
- **Tested**: Production code for line following, stair climbing, etc.

## Technical Details

### DSP Parameters Format
```python
dsp = [duration_ms, phase1, phase2]
# Example: [300, 0.2, 0.02]
# - 300ms total double support time
# - 0.2, 0.02 are phase timing ratios
```

### Hip Pitch Offset
- **Purpose**: Tilts upper body forward 15°
- **Effect**: Better balance, prevents backward falling
- **Critical**: Essential for dynamic walking

### Z Swap Amplitude
- **Purpose**: Vertical body oscillation during weight transfer
- **Effect**: Natural hip sway, smooth weight distribution
- **Value**: 0.006m (6mm) - subtle but important

### Arm Swing
- **Purpose**: Counter-rotation for balance
- **Effect**: More human-like, better stability
- **Value**: 30° swing amplitude

## Expected Results

### Motion Quality
✅ **Smooth, continuous walking** (no jerky stops)
✅ **Natural cadence** (human-like rhythm)
✅ **Stable turning** (no wobbling)
✅ **Balanced posture** (no falling)
✅ **Natural arm swing** (counter-balancing)

### Rectangle Accuracy
- **30cm long sides**: ~30 steps @ 10mm/step
- **20cm short sides**: ~20 steps @ 10mm/step
- **90° turns**: 15 steps @ 6°/step

### Performance
- **Faster completion**: ~66% faster than previous version
- **Smoother motion**: No jerky movements
- **More stable**: Better balance parameters

## Testing
```bash
# Source workspace
source ~/competition_ws/devel/setup.bash

# Launch rectangle walker
roslaunch line_follower_test rectangle_walker.launch

# Optional: Adjust parameters
roslaunch line_follower_test rectangle_walker.launch \
  side_long:=0.40 \
  side_short:=0.25 \
  num_laps:=3
```

## Comparison Table

| Parameter | Old (Jerky) | New (Smooth) | Source |
|-----------|-------------|--------------|--------|
| Forward DSP | 600ms | 300ms | Reference |
| Turn DSP | 700ms | 400ms | Reference |
| Step Height | 0.010m | 0.015m | Reference |
| Hip Offset | 15° | 15° | Reference |
| Z Swap | 0.004m | 0.006m | Reference |
| Arm Swing | 0° | 30° | Reference |
| Forward Speed | 6mm | 10mm | Tuned |
| Turn Speed | 8° | 6° | Tuned |

## Credits
- **Reference Implementation**: Ainex Example Package (visual_patrol.py)
- **Original Author**: aiden @ 2023/07/06
- **Adapted For**: Rectangle walking demonstration

---
*Last Updated: 2025-10-28*
