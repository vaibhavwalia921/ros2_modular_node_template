#!/bin/bash

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}Building ROS 2 Template Node...${NC}"

# Source ROS 2
source /opt/ros/humble/setup.bash

# Clean build directory if requested
if [ "$1" == "clean" ]; then
    echo -e "${YELLOW}Cleaning build directories...${NC}"
    rm -rf build install log
fi

# Build the project
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

if [ $? -ne 0 ]; then
    echo -e "${RED}Build failed!${NC}"
    exit 1
fi

echo -e "${GREEN}Build successful!${NC}"

# Run tests if requested
if [ "$1" == "test" ] || [ "$2" == "test" ]; then
    echo -e "${YELLOW}Running tests...${NC}"
    colcon test
    colcon test-result --verbose
fi

# Source the workspace
source install/setup.bash

# Launch the node if requested
if [ "$1" == "run" ] || [ "$2" == "run" ]; then
    echo -e "${GREEN}Launching template node...${NC}"
    ros2 launch template_node template_node.launch.py
fi