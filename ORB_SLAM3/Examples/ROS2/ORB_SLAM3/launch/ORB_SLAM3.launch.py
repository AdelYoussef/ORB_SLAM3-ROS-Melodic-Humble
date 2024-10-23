#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
import os 
def generate_launch_description():

    PATH = os.getenv("ORBSLAM3_PATH")
    LD = LaunchDescription()

    # Pass arguments to the C++ node
    Listener_Node = Node(
        package = "ORB_SLAM3",
        executable = "mono",                # C++ executable
        name = "ORB_SLAM3_CPP",
        output = "screen",
        arguments = [f"{PATH}/Vocabulary/ORBvoc.txt", f"{PATH}/config/cam.yaml"]
    )

    Talker_Node = Node(
        package = "ORB_SLAM3",
        executable = "camera_publisher.py", # Python executable
        name = "ORB_SLAM3_PY",
        output = "screen"
    )

    LD.add_action(Listener_Node)
    LD.add_action(Talker_Node)

    return LD
