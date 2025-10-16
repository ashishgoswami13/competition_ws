#!/bin/bash
# Quick test script for line follower fixes

echo "======================================"
echo "Line Follower Fix Verification Script"
echo "======================================"
echo ""

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Source workspace
source /home/ubuntu/competition_ws/devel/setup.bash

echo "Checking ROS master..."
if rostopic list &> /dev/null; then
    echo -e "${GREEN}✓ ROS master is running${NC}"
else
    echo -e "${RED}✗ ROS master not running!${NC}"
    echo "Please start Gazebo first:"
    echo "  roslaunch ainex_line_follower line_follower_simulation.launch"
    exit 1
fi

echo ""
echo "Checking topics..."

# Check camera topic
if rostopic list | grep -q "/camera/rgb/image_raw"; then
    echo -e "${GREEN}✓ Camera topic exists${NC}"
    
    # Check if publishing
    timeout 2 rostopic hz /camera/rgb/image_raw &> /tmp/camera_hz.txt
    if grep -q "average rate" /tmp/camera_hz.txt; then
        RATE=$(grep "average rate" /tmp/camera_hz.txt | awk '{print $3}')
        echo -e "${GREEN}  Camera publishing at ${RATE} Hz${NC}"
    else
        echo -e "${YELLOW}  Camera topic exists but may not be publishing${NC}"
    fi
else
    echo -e "${RED}✗ Camera topic not found${NC}"
fi

# Check debug image topic
if rostopic list | grep -q "/line_follower_node/debug_image"; then
    echo -e "${GREEN}✓ Debug image topic exists${NC}"
else
    echo -e "${RED}✗ Debug image topic not found${NC}"
fi

# Check line detection topic
if rostopic list | grep -q "/line_follower_node/line_detected"; then
    echo -e "${GREEN}✓ Line detection topic exists${NC}"
else
    echo -e "${RED}✗ Line detection topic not found${NC}"
fi

echo ""
echo "Checking controllers..."

# Check if controller manager is available
if rosservice list | grep -q "/controller_manager/list_controllers"; then
    echo -e "${GREEN}✓ Controller manager available${NC}"
    
    # Check head controllers
    CONTROLLERS=$(rosservice call /controller_manager/list_controllers 2>/dev/null)
    
    if echo "$CONTROLLERS" | grep -q "head_tilt_controller"; then
        echo -e "${GREEN}  ✓ head_tilt_controller loaded${NC}"
    else
        echo -e "${RED}  ✗ head_tilt_controller NOT loaded${NC}"
    fi
    
    if echo "$CONTROLLERS" | grep -q "head_pan_controller"; then
        echo -e "${GREEN}  ✓ head_pan_controller loaded${NC}"
    else
        echo -e "${RED}  ✗ head_pan_controller NOT loaded${NC}"
    fi
else
    echo -e "${YELLOW}⚠ Controller manager not available${NC}"
fi

echo ""
echo "Checking joint states..."
timeout 2 rostopic echo /joint_states -n 1 > /tmp/joint_states.txt 2>&1
if grep -q "head_tilt" /tmp/joint_states.txt; then
    echo -e "${GREEN}✓ Head joints present in joint_states${NC}"
    
    # Extract head_tilt position
    TILT_POS=$(grep -A 20 "name:" /tmp/joint_states.txt | grep -A 20 "head_tilt" | grep "position:" | head -1 | awk '{print $2}')
    if [ ! -z "$TILT_POS" ]; then
        echo -e "${GREEN}  head_tilt position: ${TILT_POS} rad${NC}"
    fi
else
    echo -e "${YELLOW}⚠ Head joints not found in joint_states${NC}"
fi

echo ""
echo "======================================"
echo "Summary"
echo "======================================"

echo ""
echo "To view the camera feed:"
echo "  rosrun image_view image_view image:=/camera/rgb/image_raw"
echo ""
echo "To view the debug image with line detection:"
echo "  rosrun image_view image_view image:=/line_follower_node/debug_image"
echo ""
echo "To manually adjust head tilt:"
echo "  rostopic pub /head_tilt_controller/command std_msgs/Float64 \"data: 0.5\""
echo ""
echo "To check line detection status:"
echo "  rostopic echo /line_follower_node/line_detected"
echo ""

# Clean up
rm -f /tmp/camera_hz.txt /tmp/joint_states.txt
