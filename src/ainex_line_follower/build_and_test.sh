#!/bin/bash
# Complete build and test script for ainex_line_follower

echo "=================================================="
echo "  Ainex Line Follower - Build & Test Script"
echo "=================================================="
echo ""

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Navigate to workspace
cd /home/ubuntu/competition_ws

echo "Step 1: Building the package..."
echo "-------------------------------"
catkin build ainex_line_follower

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Build successful!${NC}"
else
    echo -e "${RED}✗ Build failed!${NC}"
    exit 1
fi

echo ""
echo "Step 2: Checking package files..."
echo "----------------------------------"

# Check if all important files exist
FILES=(
    "src/ainex_line_follower/package.xml"
    "src/ainex_line_follower/CMakeLists.txt"
    "src/ainex_line_follower/scripts/line_follower_node.py"
    "src/ainex_line_follower/scripts/simple_camera_publisher.py"
    "src/ainex_line_follower/config/line_follower_params.yaml"
    "src/ainex_line_follower/launch/line_follower_simulation.launch"
)

ALL_GOOD=true
for file in "${FILES[@]}"; do
    if [ -f "$file" ]; then
        echo -e "${GREEN}✓${NC} $file"
    else
        echo -e "${RED}✗${NC} $file (missing)"
        ALL_GOOD=false
    fi
done

if [ "$ALL_GOOD" = false ]; then
    echo -e "${RED}Some files are missing!${NC}"
    exit 1
fi

echo ""
echo "Step 3: Sourcing workspace..."
echo "-----------------------------"
source devel/setup.bash
echo -e "${GREEN}✓ Workspace sourced${NC}"

echo ""
echo "Step 4: Verifying package..."
echo "----------------------------"
rospack find ainex_line_follower > /dev/null 2>&1

if [ $? -eq 0 ]; then
    PACKAGE_PATH=$(rospack find ainex_line_follower)
    echo -e "${GREEN}✓ Package found at: $PACKAGE_PATH${NC}"
else
    echo -e "${RED}✗ Package not found in ROS package path${NC}"
    exit 1
fi

echo ""
echo "=================================================="
echo -e "${GREEN}  Setup Complete! ✓${NC}"
echo "=================================================="
echo ""
echo "Next Steps:"
echo ""
echo "1. Source your workspace (in every new terminal):"
echo "   ${YELLOW}source ~/competition_ws/devel/setup.bash${NC}"
echo ""
echo "2. Launch the full simulation:"
echo "   ${YELLOW}roslaunch ainex_line_follower line_follower_simulation.launch${NC}"
echo ""
echo "3. (Optional) View debug images:"
echo "   ${YELLOW}rosrun image_view image_view image:=/line_follower/debug_image${NC}"
echo ""
echo "4. (Optional) Test without Gazebo:"
echo "   ${YELLOW}roslaunch ainex_line_follower test_line_follower.launch${NC}"
echo ""
echo "For more information, see:"
echo "  - README.md - Full documentation"
echo "  - USAGE.md - Quick start guide"
echo "  - SUMMARY.md - Package overview"
echo ""
