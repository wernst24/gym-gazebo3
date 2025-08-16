#!/bin/bash

# Simple activation script for gym-gazebo3 development environment
# Usage: source activate_env_simple.sh

echo "Activating gym-gazebo3 development environment (simplified)..."

# Source ROS2 Kilted
if [ -f "/opt/ros/kilted/setup.bash" ]; then
    source /opt/ros/kilted/setup.bash
    echo "✓ ROS2 Kilted sourced (ROS_DISTRO: $ROS_DISTRO)"
else
    echo "⚠ ROS2 Kilted not found at /opt/ros/kilted/setup.bash"
    echo "  Install with: sudo apt install ros-kilted-desktop"
fi

# Source the local workspace if built
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
    echo "✓ Local ROS2 workspace sourced"
else
    echo "⚠ Local workspace not built yet."
    echo "  Run: colcon build --packages-select gym_gazebo3"
fi

# Set Gazebo resource paths for model discovery
if [ -d "$HOME/.gazebo/models" ]; then
    export GZ_SIM_RESOURCE_PATH="$HOME/.gazebo/models:${GZ_SIM_RESOURCE_PATH:-}"
    echo "✓ Gazebo model path configured"
else
    echo "⚠ Gazebo models not found at ~/.gazebo/models"
    echo "  Switch to 'subprocess-gazebo-implementation' branch for full setup"
fi

# Test import
if python3 -c "import gymnasium; import gym_gazebo3" 2>/dev/null; then
    echo "✅ Environment setup complete!"
else
    echo "❌ Import test failed. Install dependencies: pip3 install --user gymnasium numpy"
fi

echo ""
echo "Available environments:"
echo "  • GazeboEmptySimple-v0       - Simple test environment"
echo "  • GazeboTurtlebot3Camera-v0  - TurtleBot3 with depth camera (matches target repo)"
echo ""
echo "Usage examples:"
echo "  1. Simple test: python3 examples/test_simple_env.py"
echo "  2. TurtleBot3: python3 examples/test_turtlebot3_camera_random.py"
echo ""
echo "Manual Gazebo launch:"
echo "  ros2 launch gym_gazebo3 empty_gazebo_launch.py headless:=true"
echo "  ros2 launch gym_gazebo3 turtlebot3_camera_launch.py headless:=true"
echo ""
echo "For full subprocess approach: git checkout subprocess-gazebo-implementation"