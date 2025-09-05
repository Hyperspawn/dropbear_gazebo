#!/bin/bash

# Dropbear Gazebo Workspace Setup Script
# This script helps set up the ROS 2 workspace for the Dropbear humanoid robot

echo "🤖 Setting up Dropbear Gazebo Workspace..."

# Check if ROS 2 is installed
if ! command -v ros2 &> /dev/null; then
    echo "❌ ROS 2 is not installed. Please install ROS 2 first."
    echo "   Visit: https://docs.ros.org/en/humble/Installation.html"
    exit 1
fi

# Check if colcon is installed
if ! command -v colcon &> /dev/null; then
    echo "📦 Installing colcon..."
    sudo apt update
    sudo apt install python3-colcon-common-extensions
fi

# Install required dependencies
echo "📦 Installing required dependencies..."
sudo apt update
sudo apt install -y \
    ros-humble-gazebo-ros2-control \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-controller-manager \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers

# Build the workspace
echo "🔨 Building workspace..."
colcon build

if [ $? -eq 0 ]; then
    echo "✅ Build successful!"
    
    # Source the workspace
    echo "📂 Sourcing workspace..."
    source install/setup.bash
    
    echo ""
    echo "🎉 Setup complete! Your Dropbear Gazebo workspace is ready."
    echo ""
    echo "📋 Next steps:"
    echo "   1. Launch simulation: ros2 launch dropbear_granular_urdf hyperspawn_dropbear_gazebo.launch.py"
    echo "   2. Launch RViz: ros2 launch dropbear_granular_urdf hyperspawn_dropbear_display.launch.py"
    echo ""
    echo "📖 For more information, see the README.md file."
else
    echo "❌ Build failed. Please check the error messages above."
    exit 1
fi 