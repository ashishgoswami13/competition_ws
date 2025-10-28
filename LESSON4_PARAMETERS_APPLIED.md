# Lesson 4 Tutorial Parameters - Applied to Both Packages

## Summary
Successfully applied **Lesson 4 Gazebo Simulation** tutorial default parameters to **both** line follower packages:
- ✅ `ainex_line_follower` (main stable package)
- ✅ `line_follower_test` (experimental package)

Both now use identical, proven simulation parameters from the official tutorial.

## Parameters Applied

### Source
**Reference**: `Reference/ainex_driver/ainex_kinematics/config/walking_param_sim.yaml`
**Tutorial**: `Reference/Lesson 4 Gazebo Simulation & Motion Planning/input.md`
**Controller**: `Reference/main.py`

### Gait Parameters (Both Packages)

```python
# Forward walking gait
go_gait_param = {
    'body_height': 0.025,          # init_z_offset (tutorial default)
    'step_height': 0.015,          # z_move_amplitude (tutorial default)
    'hip_pitch_offset': 15,        # hip_pitch_offset (tutorial default)
    'z_swap_amplitude': 0.006,     # z_swap_amplitude (tutorial default)
    'pelvis_offset': 0             # pelvis_offset (tutorial default)
}

# Turning gait
turn_gait_param = {
    'body_height': 0.025,
    'step_height': 0.020,          # Slightly higher for turns
    'hip_pitch_offset': 15,
    'z_swap_amplitude': 0.006,
    'pelvis_offset': 0
}

# DSP parameters [period_ms, dsp_ratio, y_swap_amplitude]
go_dsp = [1500, 0.3, 0.035]       # EXACT tutorial simulation defaults
turn_dsp = [1800, 0.35, 0.035]    # Slower for turning stability

# Arm swing
go_arm_swap = 0                    # Disabled in simulation
turn_arm_swap = 0
```

### Speed Parameters

| Package | Forward Speed | Turn Speed | Source |
|---------|---------------|------------|--------|
| **ainex_line_follower** | 0.05m | 10° | Tutorial example |
| **line_follower_test** | 0.05m | 10° | Tutorial example |

## Files Updated

### ainex_line_follower Package
```
src/ainex_line_follower/
├── scripts/
│   └── line_follower_node_v2.py        ✅ Updated gait parameters
└── config/
    └── line_follower_params.yaml        ✅ Updated speed parameters
```

### line_follower_test Package
```
src/line_follower_test/
├── scripts/
│   └── rectangle_walker_node.py        ✅ Updated gait parameters
├── launch/
│   └── rectangle_walker.launch         ✅ Updated launch parameters
└── [Documentation files]                ✅ Added tutorial reference docs
```

## Key Changes from Previous Version

### Timing (Most Critical)
| Parameter | Before | After (Tutorial) | Change |
|-----------|--------|------------------|--------|
| `period_time` (walk) | 600ms | **1500ms** | 2.5× slower |
| `period_time` (turn) | 700ms | **1800ms** | 2.6× slower |
| `dsp_ratio` | 0.3 | **0.3** | Same |
| `y_swap_amplitude` | 0.015m | **0.035m** | 2.3× larger |

### Speed
| Parameter | Before | After (Tutorial) | Change |
|-----------|--------|------------------|--------|
| `forward_speed` | 0.006m | **0.05m** | 8.3× faster steps |
| `turn_speed` | 6° | **10°** | 1.7× faster |

### Gait
| Parameter | Before | After (Tutorial) | Change |
|-----------|--------|------------------|--------|
| `step_height` (walk) | 0.008m | **0.015m** | 1.9× higher |
| `step_height` (turn) | 0.010m | **0.020m** | 2× higher |
| `z_swap_amplitude` | 0.004m | **0.006m** | 1.5× more oscillation |
| `arm_swing` | 0° | **0°** | Same (disabled) |

## Why These Changes Matter

### 1. **Slower Period = More Stable**
- `period_time: 1500ms` gives Gazebo physics engine time to calculate
- Prevents physics simulation errors
- Works on slower computers (as per tutorial warning about FPS)

### 2. **Larger Steps = Fewer Steps**
- `forward_speed: 0.05m` means bigger steps
- Completes distance in fewer steps
- Tutorial example shows this exact value

### 3. **More Body Sway = Better Balance**
- `y_swap_amplitude: 0.035m` provides better weight transfer
- Simulation needs exaggerated motion vs real robot
- Prevents tipping in virtual physics

### 4. **Higher Step Lift = Clearer Motion**
- `step_height: 0.015/0.020m` makes steps more visible
- Helps robot clear obstacles in simulation
- More natural-looking gait

## Usage

### Line Follower (ainex_line_follower)
```bash
source ~/competition_ws/devel/setup.bash
roslaunch ainex_line_follower line_follower_simulation.launch
```

### Rectangle Walker (line_follower_test)
```bash
source ~/competition_ws/devel/setup.bash
roslaunch line_follower_test rectangle_walker.launch
```

## Expected Behavior

### Line Follower
- **Slower, deliberate motion** (1.5s per step)
- **Larger forward steps** (5cm vs 6mm)
- **Smoother turns** (10° per step)
- **More stable** (won't fall easily)
- **Tutorial-compliant** (matches Lesson 4 example)

### Rectangle Walker
- **Slow rectangle tracing** (~2-3 minutes per 30×20cm lap)
- **Visible step movements** (easier to debug)
- **Stable corners** (slower turns)
- **Predictable timing** (1.5-1.8s per step)

## Tutorial Compliance Checklist

✅ **period_time**: 1500ms (matches walking_param_sim.yaml)
✅ **dsp_ratio**: 0.3 (matches walking_param_sim.yaml)
✅ **y_swap_amplitude**: 0.035m (matches walking_param_sim.yaml)
✅ **z_move_amplitude**: 0.015m (matches walking_param_sim.yaml)
✅ **hip_pitch_offset**: 15° (matches walking_param_sim.yaml)
✅ **z_swap_amplitude**: 0.006m (matches walking_param_sim.yaml)
✅ **pelvis_offset**: 0° (matches walking_param_sim.yaml)
✅ **arm_swing_gain**: 0.0 (matches walking_param_sim.yaml)
✅ **x_move_amplitude**: 0.05m (matches Lesson 4 example)

## Performance Notes

From Lesson 4 Tutorial:
> "After starting the simulation, keep an eye on the frame rate (FPS) displayed beneath the Gazebo simulation window. If it drops below '9' FPS, you should consider using a more powerful computer."

**These parameters are optimized for**:
- Gazebo physics simulation
- Computers with FPS ≥ 9
- Smooth, stable motion
- Tutorial compliance

## Verification

Both packages now use **identical base parameters**:
```bash
# Check parameters match tutorial
grep -A 5 "go_dsp = " src/ainex_line_follower/scripts/line_follower_node_v2.py
grep -A 5 "go_dsp = " src/line_follower_test/scripts/rectangle_walker_node.py

# Both should show: [1500, 0.3, 0.035]
```

---
*Synchronized with Lesson 4 Tutorial: 2025-10-28*
*Source: Reference/Lesson 4 Gazebo Simulation & Motion Planning*
