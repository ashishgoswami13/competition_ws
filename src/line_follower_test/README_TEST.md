# Line Follower Test Package

## Purpose
This is an experimental package created as a copy of `ainex_line_follower` for testing and developing improvements to the line following algorithm.

## What's Different from ainex_line_follower?
This package allows you to:
- Test new algorithms without breaking the original working version
- Experiment with different PID parameters
- Try alternative control strategies
- Develop new features independently

## Current Features (Inherited from ainex_line_follower)
- ✅ PID-based turn control for smooth steering
- ✅ Dual gait system (separate parameters for straight/turning)
- ✅ Adaptive speed reduction on sharp turns
- ✅ Camera calibration offset support
- ✅ Reference-based 3-zone control (fallback)
- ✅ Line-loss recovery with forward movement
- ✅ Comprehensive debug visualization

## How to Use

### Launch the Test Package
```bash
# Source the workspace
source ~/competition_ws/devel/setup.bash

# Launch the test simulation
roslaunch line_follower_test line_follower_simulation.launch
```

### Compare with Original
```bash
# Launch original version
roslaunch ainex_line_follower line_follower_simulation.launch

# Launch test version  
roslaunch line_follower_test line_follower_simulation.launch
```

## Key Files to Modify for Experiments

### 1. Main Script
`scripts/line_follower_node_v2.py`
- PID controller implementation
- Turn angle calculation
- Walking control logic

### 2. Configuration
`config/line_follower_params.yaml`
- PID gains (Kp, Ki, Kd)
- Turn angle limits
- Speed parameters
- Camera calibration

### 3. Launch File
`launch/line_follower_simulation.launch`
- Spawn position
- Simulation parameters

## Experimenting with PID

### Current PID Values
```yaml
pid_kp: 0.08   # Proportional gain
pid_ki: 0.001  # Integral gain  
pid_kd: 0.15   # Derivative gain
```

### Tuning Tips
- **Too much oscillation?** → Increase Kd or decrease Kp
- **Too slow response?** → Increase Kp
- **Persistent offset?** → Increase Ki (but keep small!)
- **Unstable?** → Decrease all gains by 50%, then tune up

## Testing Workflow
1. Make changes in `line_follower_test`
2. Rebuild: `catkin build line_follower_test`
3. Source: `source ~/competition_ws/devel/setup.bash`
4. Test: `roslaunch line_follower_test line_follower_simulation.launch`
5. Compare with original if needed

## Preserving Working Version
The original `ainex_line_follower` package remains unchanged, so you always have a working baseline to compare against or revert to.

---
**Created:** October 2025
**Based on:** ainex_line_follower v1.0.0
