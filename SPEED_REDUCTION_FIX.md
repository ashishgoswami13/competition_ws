# Stability Fix - Reduced Walking Speed

## Problem
Robot was walking too fast and falling down after a few steps due to:
- Excessive forward speed
- Too aggressive turning
- Fast walking speed causing instability
- Insufficient stability margins for humanoid balance

## Changes Made

### 1. Reduced Walking Speed (MAJOR)
**File**: `config/line_follower_params.yaml`

| Parameter | Before | After | Change |
|-----------|--------|-------|--------|
| `walking_speed` | 2 (medium) | **1 (slowest)** | -50% speed |
| `forward_speed` | 0.012 m | **0.008 m** | -33% step size |
| `turn_gain` | 0.8 | **0.6** | -25% turn sensitivity |
| `max_turn_angle` | 10.0° | **8.0°** | -20% max turn |

**Impact:**
- Walking speed 1 is the slowest and most stable option (1-4 scale)
- Smaller forward steps = better balance
- Gentler turns = less tipping
- Lower turn gain = smoother corrections

### 2. Slower First Step
**File**: `scripts/line_follower_node.py`

**Before:**
```python
self.gait_manager.move(1, 0.005, 0.0, 0, ...)  # First step
rospy.sleep(0.5)  # Brief pause
```

**After:**
```python
self.gait_manager.move(1, 0.003, 0.0, 0, ...)  # SMALLER first step
rospy.sleep(1.0)  # LONGER pause
```

**Impact:**
- First step is 40% smaller (0.005 → 0.003)
- Pause is 2x longer (0.5s → 1.0s)
- Gentler transition from standing to walking

### 3. Slower Recovery Speed
**Before:**
```python
# When line lost
self.gait_manager.move(
    self.walking_speed,      # Use normal speed
    self.forward_speed * 0.5,  # Half speed
    ...
)
```

**After:**
```python
# When line lost
self.gait_manager.move(
    1,                          # Force slowest speed
    self.forward_speed * 0.3,   # Only 30% speed
    ...
)
```

**Impact:**
- Recovery is VERY slow and stable
- Less chance of falling while searching for line
- More controlled recovery motion

## Complete Parameter Set

### Walking Parameters (All Reduced for Stability)
```yaml
forward_speed: 0.008        # meters (was 0.012)
turn_gain: 0.6              # sensitivity (was 0.8)
max_turn_angle: 8.0         # degrees (was 10.0)
walking_speed: 1            # slowest setting (was 2)
body_height: 0.025          # unchanged - matches Reference
```

### First Step Parameters
```python
speed: 1                    # slowest
forward: 0.003             # very small (was 0.005)
pause: 1.0s                # longer (was 0.5s)
```

### Recovery Parameters
```python
speed: 1                   # force slowest (was walking_speed)
forward: 0.008 * 0.3      # 30% of already-slow speed
```

## Expected Behavior

### Walking Speed Comparison
| Speed | Period | Description | Stability |
|-------|--------|-------------|-----------|
| 4 | 300ms | Fastest | Low ⚠️ |
| 3 | 400ms | Fast | Medium |
| 2 | 500ms | Medium | Good ✓ |
| **1** | **600ms+** | **Slowest** | **Excellent ✓✓** |

### Stability Hierarchy
```
Speed 1 (Slowest) → Most Stable → CURRENT SETTING ✅
  └─ Longest time per step
  └─ Best balance recovery
  └─ Least momentum
  └─ Maximum stability

Speed 2 (Medium) → Good Stability → PREVIOUS SETTING
  └─ Faster walking
  └─ Less stable than 1

Speed 3-4 (Fast) → Lower Stability → TOO RISKY
  └─ High speed = high momentum
  └─ Risk of falling
```

## Testing Checklist

After launching, observe:

### ✅ Spawn & Initialization
- [ ] Robot spawns upright
- [ ] No falling during 3s pause
- [ ] Smooth unpause of physics
- [ ] Stable standing pose

### ✅ First Step (t=9s)
- [ ] Very small, slow first step
- [ ] 1-second pause after
- [ ] Robot remains balanced
- [ ] No jerking

### ✅ Line Following
- [ ] SLOW walking speed (clearly slower than before)
- [ ] Small forward steps
- [ ] Gentle turning
- [ ] Robot stays upright for 30+ seconds
- [ ] Smooth motion along line

### ✅ Recovery (if line lost)
- [ ] Even slower recovery motion
- [ ] No aggressive movements
- [ ] Stable search pattern

## Troubleshooting

### If robot still falls:
1. **Check spawn height**: Should be 0.25m
2. **Verify paused start**: Gazebo should start frozen
3. **Watch first few steps**: Should be very slow and deliberate
4. **Check for obstacles**: Ensure clear path on line

### If too slow (but stable):
Once stable, can gradually increase:
1. First try `walking_speed: 2` (keep other params)
2. Then try `forward_speed: 0.010`
3. Then try `turn_gain: 0.7`
4. Always test stability after each change

### If turning issues persist:
We'll address turning logic separately after confirming:
- Robot walks stably in straight line ✓
- Robot doesn't fall ✓
- Speed is acceptable ✓

## Performance vs Stability Trade-off

```
Current Settings: MAXIMUM STABILITY MODE

Speed: ████░░░░░░ 10%  (Slowest)
Stability: ██████████ 100% (Maximum)

Previous Settings: BALANCED MODE
Speed: ████████░░ 80%  (Medium-fast) 
Stability: ██████░░░░ 60% (Falls after few steps)

Goal: Find sweet spot between speed and stability
```

## Next Steps (After Testing)

1. **Test this version** - Confirm no falling
2. **If stable**: We can cautiously increase speed slightly
3. **If still unstable**: Further reduce forward_speed to 0.006
4. **Then**: Fix turning logic (separate issue)

## Build & Test

```bash
cd ~/competition_ws
catkin build ainex_line_follower
source devel/setup.bash
roslaunch ainex_line_follower line_follower_simulation.launch
```

**Watch for:**
- MUCH slower walking (should be obvious)
- Very small, deliberate steps
- Gentle, smooth motion
- **Robot stays upright!** ✅

## Date Modified
October 17, 2025

## Status
✅ **REBUILT - READY FOR TESTING**

Please test and report if robot:
1. Walks stably without falling ✅/❌
2. Speed is acceptable (slower but stable) ✅/❌
3. Can follow line for 30+ seconds ✅/❌
