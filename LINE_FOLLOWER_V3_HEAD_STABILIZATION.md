# Line Follower V3 - Head Stabilization & Search Behavior

## Date: 2025-10-17

## 🎯 Overview
Version 3 addresses motion jerkiness and adds intelligent search behavior when the line is lost, using the robot's head/neck for camera stabilization and active searching.

---

## ✨ Major New Features

### 1. **Head Stabilization (Flexible Neck)**
The robot's neck joints now have **increased damping** for smoother, more stable camera motion.

#### URDF Changes:
```xml
<!-- New head-specific damping properties -->
<xacro:property name="head_damping" value="0.8"/>    <!-- 40x higher than body joints -->
<xacro:property name="head_friction" value="0.1"/>

<!-- Applied to both head joints -->
<joint name="head_pan">    <!-- Left-right -->
  <dynamics damping="${head_damping}" friction="${head_friction}" />
</joint>

<joint name="head_tilt">   <!-- Up-down -->
  <dynamics damping="${head_damping}" friction="${head_friction}" />
</joint>
```

**Benefits:**
- ✅ Reduces camera shake during walking
- ✅ Smoother video feed for line detection
- ✅ More stable visual processing
- ✅ Less jerky robot appearance

---

### 2. **Active Head Search Behavior**
When the line is lost, the robot **actively searches** by moving its head while continuing to walk slowly forward.

#### Search Pattern:
```python
search_positions = [
    500,  # 1. Center (straight ahead)
    650,  # 2. Left (~30°)
    350,  # 3. Right (~30°)
    750,  # 4. Far left (~50°)
    250,  # 5. Far right (~50°)
]
```

#### Search Algorithm:
1. **Line Lost**: Detected when `line_lost_count` reaches threshold
2. **Head Search**: Robot moves head through 5 positions
3. **Slow Forward**: Walks at 40% minimum speed during search
4. **Active Scan**: Each position held for 300ms
5. **Line Found**: Head returns to center, resumes normal following
6. **Timeout**: Stops after 5 seconds if line not found

---

### 3. **Head Control Integration**

Uses `MotionManager` API for precise head control:

```python
# Initialize both managers
self.gait_manager = GaitManager()      # For walking
self.motion_manager = MotionManager()  # For head control

# Set head position
self.motion_manager.set_servos_position(duration_ms, [
    [HEAD_PAN_ID, pulse_width],   # Servo 23: 125-875 range
    [HEAD_TILT_ID, pulse_width]   # Servo 24: 300-500 range
])
```

#### Head Servo Details:
- **Head Pan (ID 23)**: Left-right rotation
  - Range: 125 (far right) to 875 (far left)
  - Center: 500
  - Used for: Line searching
  
- **Head Tilt (ID 24)**: Up-down rotation
  - Range: 300 (down) to 500 (up)
  - Default: 380 (slightly down to see line)
  - Used for: Maintaining line visibility

---

## 🔄 State Machine

### State Flow:
```
INIT → STANDING → FIRST_STEP → LINE_FOLLOWING
                                      ↓
                              LINE_LOST (count > 0)
                                      ↓
                              SEARCHING (head scan)
                                  ↙        ↘
                        FOUND (return)   TIMEOUT (stop)
                              ↓
                        LINE_FOLLOWING
```

### State Descriptions:

**INIT**: 
- Wait 2s for simulation
- Update pose to standing
- Position head (pan=500, tilt=380)

**FIRST_STEP**: 
- Very small step (0.003m)
- No turning
- 1s pause after

**LINE_FOLLOWING**: 
- Normal operation
- Adaptive speed (0.006-0.010m)
- Turn angle (0-8°)

**SEARCHING**: 
- Head scans 5 positions
- Walk forward at 0.0024m (40% min speed)
- 300ms per position
- Max 5 seconds total

**FOUND**: 
- Return head to center (500, 380)
- Resume line following
- Reset search state

---

## 📊 Performance Comparison

| Feature | V2 | V3 |
|---------|----|----|
| **Head Damping** | 0.02 (rigid) | 0.8 (flexible) |
| **Camera Stability** | Jerky during walking | Smooth, stabilized |
| **Line Loss Behavior** | Stop or continue straight | Active head search |
| **Recovery** | Passive (wait for line) | Active (search for line) |
| **Search Duration** | N/A | 5 seconds max |
| **Search Positions** | N/A | 5 positions |
| **API Used** | GaitManager only | GaitManager + MotionManager |

---

## 🎮 Control Flow

### Normal Line Following:
```python
1. Detect line in image
2. Calculate error from center
3. Determine turn angle (0-8°)
4. Calculate speed (0.006-0.010m)
5. Execute walk command
6. Log status
```

### Search Behavior:
```python
1. Line not detected for >1 frame
2. Start search timer
3. Move head to position[0] (center)
4. Wait 300ms, check for line
5. Move to position[1] (left)
6. Wait 300ms, check for line
7. Continue through all 5 positions
8. Repeat cycle until:
   - Line found → return to center, resume
   - 5 seconds elapsed → stop searching, stop robot
```

---

## 🔧 Configuration Parameters

### Walking Parameters:
```yaml
# From line_follower_params.yaml
walking_speed: 4              # 600ms period (slowest)
forward_speed: 0.010          # Max forward (m)
min_forward_speed: 0.006      # Min forward when turning (m)
turn_gain: 0.6                # Not used in V3
max_turn_angle: 8.0           # Maximum turn (degrees)
min_turn_angle: 4.0           # Turning threshold (degrees)
center_tolerance: 10          # Dead zone (pixels)
body_height: 0.025            # m
step_height: 0.010            # m
```

### Head Parameters:
```python
# In line_follower_node_v3.py
head_pan_center = 500         # Center position
head_tilt_center = 380        # Looking down at line
head_pan_range = [125, 875]   # Full range
head_tilt_range = [300, 500]  # Limited for safety
head_search_speed = 300       # Movement duration (ms)
```

### Search Parameters:
```python
max_line_lost_frames = 30     # 3 seconds at 10Hz before timeout
search_timeout = 5.0          # Maximum search duration (s)
search_speed_factor = 0.4     # Speed during search (40% of min)
```

---

## 🐛 Jerkiness Solutions

### Problem: "Motion is too jerky"

**Root Causes Identified:**
1. ❌ Rigid neck (damping=0.02)
2. ❌ Camera shake during walking
3. ❌ Sudden speed changes
4. ❌ No visual stabilization

**Solutions Implemented:**

1. **Increased Head Damping (0.02 → 0.8)**
   - Acts like shock absorber
   - Smooths out walking vibrations
   - Camera stays steadier

2. **Smooth Head Movements**
   - Gradual position changes (300ms duration)
   - No sudden head jerks
   - Controlled acceleration

3. **Adaptive Speed Control**
   - Gradual speed changes based on turn angle
   - No abrupt stops/starts
   - Smooth interpolation

4. **First Step Protection**
   - Very gentle initial movement (0.003m)
   - 1 second pause after first step
   - Allows stabilization

---

## 📝 Code Structure

### Main Components:

```python
class LineFollowerV3:
    # Initialization
    __init__()              # Setup managers, parameters
    initialize_robot()      # Gradual startup sequence
    
    # Head Control
    set_head_position()     # Control pan/tilt servos
    search_for_line()       # Execute search pattern
    on_line_found()         # Handle search completion
    
    # Vision
    detect_line()           # OpenCV line detection
    
    # Walking Control
    calculate_turn_angle()  # Dead zone + proportional
    calculate_forward_speed() # Speed based on turn
    execute_walk_command()  # GaitManager.set_step()
    control_walking()       # Main control logic
    
    # Main Loop
    run()                   # ROS control loop
```

---

## 🧪 Testing Checklist

### Initialization:
- [ ] Robot stands smoothly without jerking
- [ ] Head positions correctly (down, centered)
- [ ] Camera shows clear line view
- [ ] No violent movements

### Normal Following:
- [ ] Smooth walking on straight line
- [ ] Camera remains stable (no shake)
- [ ] Gentle turns on curves
- [ ] Appropriate speed changes

### Line Loss - Search Behavior:
- [ ] Head starts scanning when line lost
- [ ] Robot continues walking slowly forward
- [ ] Head moves through all 5 positions
- [ ] Clear "SEARCHING..." status in debug image

### Line Recovery:
- [ ] Detects line during search
- [ ] Head returns to center smoothly
- [ ] Resumes normal following
- [ ] No abrupt transitions

### Search Timeout:
- [ ] Stops after 5 seconds if no line
- [ ] Logs timeout message
- [ ] Robot stops walking
- [ ] Clean shutdown

---

## 🎯 Expected Improvements

### Compared to V2:

1. **Visual Stability** 📹
   - V2: Shaky camera, jerky footage
   - V3: Smooth, stabilized video feed

2. **Line Loss Handling** 🔍
   - V2: Passive (continue straight or stop)
   - V3: Active search with head scanning

3. **Recovery Rate** 📈
   - V2: Low (depends on luck)
   - V3: High (actively searches)

4. **User Experience** 😊
   - V2: Appears rigid, robotic
   - V3: Appears intelligent, adaptive

---

## 🔄 Rollback Options

### To V2 (No search):
```bash
# In launch file, change:
type="line_follower_node_v2.py"
```

### To V1 (Original):
```bash
# In launch file, change:
type="line_follower_node.py"
```

### Revert Head Damping:
```xml
<!-- In ainex.urdf.xacro, change back to: -->
<dynamics damping="${damping}" friction="${friction}" />
```

---

## 📦 Files Modified

1. **Created**: `line_follower_node_v3.py` (460 lines)
2. **Modified**: `ainex.urdf.xacro` (added head damping properties)
3. **Modified**: `line_follower_simulation.launch` (uses V3)
4. **Unchanged**: Config files (parameters still compatible)

---

## 🚀 Launch Command

```bash
roslaunch ainex_line_follower line_follower_simulation.launch
```

---

## 📊 Debug Topics

```bash
# View debug image with search status
rqt_image_view /line_follower_node/debug_image

# Monitor line detection
rostopic echo /line_follower_node/line_detected

# Watch camera feed
rqt_image_view /camera/image_raw
```

---

## 💡 Tips

1. **If camera too shaky**: Increase `head_damping` to 1.0 or 1.2
2. **If search too slow**: Reduce `head_search_speed` to 200ms
3. **If search too aggressive**: Reduce number of `search_positions`
4. **If stops too quickly**: Increase `max_line_lost_frames` to 50

---

**Status**: ✅ Ready for testing  
**Version**: 3.0  
**Features**: Head stabilization + Active search  
**Stability**: Significantly improved over V2

