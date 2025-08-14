# Frame Configuration Examples

This document provides common frame configuration examples for different robot setups.

## Standard Single Robot Setup

```bash
ros2 launch sati_ros_camera_stack sati_orbslam_stack.launch.py \
    robot_base_frame:="base_link" \
    global_frame:="map" \
    odom_frame:="odom" \
    frame_id:="camera_link"
```

## Mobile Robot with Standard ROS Frames

```bash
ros2 launch sati_ros_camera_stack sati_orbslam_stack.launch.py \
    robot_base_frame:="base_footprint" \
    global_frame:="map" \
    odom_frame:="odom" \
    frame_id:="camera_optical_frame"
```

## Multi-Robot Setup (Robot 1)

```bash
ros2 launch sati_ros_camera_stack sati_orbslam_stack.launch.py \
    robot_base_frame:="robot1/base_link" \
    global_frame:="world" \
    odom_frame:="robot1/odom" \
    frame_id:="robot1/camera"
```

## Multi-Robot Setup (Robot 2)

```bash
ros2 launch sati_ros_camera_stack sati_orbslam_stack.launch.py \
    robot_base_frame:="robot2/base_link" \
    global_frame:="world" \
    odom_frame:="robot2/odom" \
    frame_id:="robot2/camera"
```

## Drone/UAV Setup

```bash
ros2 launch sati_ros_camera_stack sati_orbslam_stack.launch.py \
    robot_base_frame:="base_link" \
    global_frame:="world" \
    odom_frame:="odom" \
    frame_id:="camera_link"
```

## Custom Coordinate System

```bash
ros2 launch sati_ros_camera_stack sati_orbslam_stack.launch.py \
    robot_base_frame:="vehicle_base" \
    global_frame:="global_map" \
    odom_frame:="vehicle_odom" \
    frame_id:="front_camera"
```

## Disable TF Publishing (External TF Source)

```bash
ros2 launch sati_ros_camera_stack sati_orbslam_stack.launch.py \
    publish_tf:=false
```

## Frame Naming Conventions

### Standard ROS Frames
- `base_link`: Main robot coordinate frame
- `base_footprint`: Robot base projected to ground plane
- `map`: Global coordinate frame
- `odom`: Odometry coordinate frame
- `camera_link`: Camera mounting frame
- `camera_optical_frame`: Camera optical center frame

### Multi-Robot Naming
- Use namespace prefixes: `robot1/base_link`, `robot2/base_link`
- Keep global frame consistent: `world` or `map`
- Separate odometry frames: `robot1/odom`, `robot2/odom`

## Checking Your TF Tree

After launching, verify your TF tree:

```bash
# View TF tree
ros2 run tf2_tools view_frames

# Check specific transform
ros2 run tf2_ros tf2_echo map base_link

# List all frames
ros2 topic echo /tf_static
```
