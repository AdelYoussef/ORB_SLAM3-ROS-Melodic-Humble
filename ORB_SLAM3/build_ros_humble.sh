#!/bin/bash

echo "Building ROS 2 nodes"

cd Examples/ROS2/ORB_SLAM3
colcon build --symlink-install --packages-select ORB_SLAM3



# Define the line you want to add
source_command="source \$ORBSLAM3_PATH/Examples/ROS2/ORB_SLAM3/install/setup.bash"

# Check if the line already exists in .bashrc
if ! grep -Fxq "$source_command" ~/.bashrc
then
    # If the line does not exist, add it to .bashrc
    echo "$source_command" >> ~/.bashrc
    echo "The line has been added to ~/.bashrc"
else
    echo "The line already exists in ~/.bashrc"
fi

# Optional: Reload the .bashrc file to apply changes
source ~/.bashrc
