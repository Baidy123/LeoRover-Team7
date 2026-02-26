#!/bin/bash

# Leo Rover LiDAR Simulation - Installation Script for ROS2 Jazzy
# This script automates the installation process

set -e  # Exit on error

echo "================================================"
echo "Leo Rover LiDAR Simulation - Installation"
echo "ROS2 Jazzy + Gazebo Harmonic"
echo "================================================"
echo ""

# Check if running Ubuntu 24.04
if [ -f /etc/os-release ]; then
    . /etc/os-release
    if [ "$VERSION_ID" != "24.04" ]; then
        echo "⚠️  Warning: This script is designed for Ubuntu 24.04"
        echo "Current version: $VERSION_ID"
        read -p "Continue anyway? (y/n) " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            exit 1
        fi
    fi
fi

# Function to check if ROS2 Jazzy is installed
check_ros2() {
    if [ -f "/opt/ros/jazzy/setup.bash" ]; then
        echo "✓ ROS2 Jazzy detected"
        return 0
    else
        echo "✗ ROS2 Jazzy not found"
        return 1
    fi
}

# Install ROS2 Jazzy
install_ros2() {
    echo ""
    echo "Installing ROS2 Jazzy..."
    echo "------------------------"
    
    sudo apt update
    sudo apt install -y software-properties-common curl
    
    # Add ROS2 repository
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    
    sudo apt update
    sudo apt install -y ros-jazzy-desktop
    
    echo "✓ ROS2 Jazzy installed"
}

# Install dependencies
install_dependencies() {
    echo ""
    echo "Installing dependencies..."
    echo "--------------------------"
    
    sudo apt update
    sudo apt install -y \
        ros-jazzy-ros-gz \
        ros-jazzy-ros-gz-sim \
        ros-jazzy-ros-gz-bridge \
        ros-jazzy-ros-gz-image \
        ros-jazzy-navigation2 \
        ros-jazzy-nav2-bringup \
        ros-jazzy-slam-toolbox \
        ros-jazzy-robot-state-publisher \
        ros-jazzy-joint-state-publisher \
        ros-jazzy-xacro \
        ros-jazzy-rviz2 \
        ros-jazzy-teleop-twist-keyboard \
        python3-pip \
        python3-numpy \
        ros-jazzy-pcl-ros \
        ros-jazzy-pcl-conversions
    
    echo "✓ Dependencies installed"
}

# Setup workspace
setup_workspace() {
    echo ""
    echo "Setting up workspace..."
    echo "-----------------------"
    
    WORKSPACE_DIR="$HOME/leo_ws"
    
    # Ask user for workspace location
    read -p "Enter workspace directory [$WORKSPACE_DIR]: " USER_WS
    WORKSPACE_DIR="${USER_WS:-$WORKSPACE_DIR}"
    
    # Create workspace
    mkdir -p "$WORKSPACE_DIR/src"
    echo "✓ Workspace created at: $WORKSPACE_DIR"
    
    # Copy package
    SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
    PACKAGE_DIR="$WORKSPACE_DIR/src/leo_lidar_sim"
    
    if [ -d "$PACKAGE_DIR" ]; then
        echo "⚠️  Package already exists at $PACKAGE_DIR"
        read -p "Overwrite? (y/n) " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            rm -rf "$PACKAGE_DIR"
        else
            echo "Skipping package copy"
            return 0
        fi
    fi
    
    cp -r "$SCRIPT_DIR" "$PACKAGE_DIR"
    echo "✓ Package copied to workspace"
    
    # Install rosdep dependencies
    cd "$WORKSPACE_DIR"
    source /opt/ros/jazzy/setup.bash
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y
    
    echo "✓ rosdep dependencies installed"
}

# Build workspace
build_workspace() {
    echo ""
    echo "Building workspace..."
    echo "---------------------"
    
    WORKSPACE_DIR="${WORKSPACE_DIR:-$HOME/leo_ws}"
    
    cd "$WORKSPACE_DIR"
    source /opt/ros/jazzy/setup.bash
    
    colcon build --symlink-install
    
    echo "✓ Workspace built successfully"
}

# Setup bashrc
setup_bashrc() {
    echo ""
    echo "Setting up .bashrc..."
    echo "---------------------"
    
    WORKSPACE_DIR="${WORKSPACE_DIR:-$HOME/leo_ws}"
    SOURCE_LINE="source $WORKSPACE_DIR/install/setup.bash"
    
    if grep -q "$SOURCE_LINE" "$HOME/.bashrc"; then
        echo "✓ .bashrc already configured"
    else
        echo "" >> "$HOME/.bashrc"
        echo "# Leo Rover Simulation Workspace" >> "$HOME/.bashrc"
        echo "$SOURCE_LINE" >> "$HOME/.bashrc"
        echo "✓ Added workspace sourcing to .bashrc"
    fi
}

# Main installation
main() {
    echo "Starting installation..."
    echo ""
    
    # Check and install ROS2
    if ! check_ros2; then
        read -p "Install ROS2 Jazzy? (y/n) " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            install_ros2
        else
            echo "❌ ROS2 Jazzy is required. Exiting."
            exit 1
        fi
    fi
    
    # Install dependencies
    read -p "Install dependencies? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        install_dependencies
    fi
    
    # Setup workspace
    read -p "Setup workspace? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        setup_workspace
        build_workspace
        setup_bashrc
    fi
    
    echo ""
    echo "================================================"
    echo "✓ Installation complete!"
    echo "================================================"
    echo ""
    echo "Next steps:"
    echo "1. Source your workspace:"
    echo "   source $WORKSPACE_DIR/install/setup.bash"
    echo ""
    echo "2. Launch the simulation:"
    echo "   ros2 launch leo_lidar_sim complete_system.launch.py"
    echo ""
    echo "3. Control the robot:"
    echo "   ros2 run teleop_twist_keyboard teleop_twist_keyboard"
    echo ""
    echo "For more information, see README.md"
    echo ""
}

# Run main installation
main
