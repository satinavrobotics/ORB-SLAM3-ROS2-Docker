#!/bin/bash
# Example script to run the complete SATI camera stack with ORB-SLAM3
# This replaces the need for the original run_orbslam.sh script

echo "Starting SATI ROS Camera Stack with ORB-SLAM3..."
echo "This will launch:"
echo "  - Camera node (camera_ros)"
echo "  - Image converter (mono8 output)"
echo "  - ORB-SLAM3 monocular SLAM"
echo ""

# Source the workspace
source /root/colcon_ws/install/setup.bash

# Launch the complete stack with default parameters
ros2 launch sati_ros_camera_stack sati_orbslam_stack.launch.py

# Alternative: Launch with custom parameters
# ros2 launch sati_ros_camera_stack sati_orbslam_stack.launch.py \
#     camera:="/base/axi/pcie@1000120000/rp1/i2c@80000/ov9281@60" \
#     format:="R8" \
#     width:=640 \
#     height:=480 \
#     fps:=30 \
#     vocabulary_path:="/home/orb/ORB_SLAM3/Vocabulary/ORBvoc.txt" \
#     settings_path:="/root/colcon_ws/src/orb_slam3_ros2_wrapper/params/arducam_mono.yaml" \
#     visualization:=false \
#     robot_base_frame:="base_link" \
#     global_frame:="map" \
#     odom_frame:="odom" \
#     frame_id:="camera_link"
