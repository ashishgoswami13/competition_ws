#!/bin/bash
# Quick setup script for ainex_line_follower package

echo "=========================================="
echo "Ainex Line Follower Package Setup"
echo "=========================================="

# Navigate to workspace
cd /home/ubuntu/competition_ws

# Build the package
echo ""
echo "Building ainex_line_follower package..."
catkin build ainex_line_follower

# Check if build was successful
if [ $? -eq 0 ]; then
    echo ""
    echo "✓ Build successful!"
    echo ""
    echo "To use the package, source your workspace:"
    echo "  source ~/competition_ws/devel/setup.bash"
    echo ""
    echo "Then launch the simulation:"
    echo "  roslaunch ainex_line_follower line_follower_simulation.launch"
    echo ""
else
    echo ""
    echo "✗ Build failed. Please check the errors above."
    exit 1
fi
