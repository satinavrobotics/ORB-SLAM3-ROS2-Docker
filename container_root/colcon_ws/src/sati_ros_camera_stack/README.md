# SATI ROS Camera Stack

A comprehensive ROS 2 camera stack that integrates the `camera_ros` package with an image converter node for seamless compatibility with ORB-SLAM3 and other vision applications.

## Features

- **Integrated Camera Support**: Built on top of the `camera_ros` package for libcamera support
- **Image Format Conversion**: Automatic conversion from various formats (mono16, bgra8, bgr8, rgb8) to mono8
- **Configurable Topics**: Parameterized input and output topics
- **Launch File Integration**: Easy-to-use launch files for different scenarios

## Package Structure

```
sati_ros_camera_stack/
├── launch/
│   ├── sati_camera_stack.launch.py    # Camera stack with converter
│   └── sati_orbslam_stack.launch.py   # Complete stack with ORB-SLAM3
├── sati_ros_camera_stack/
│   ├── __init__.py
│   └── image_converter_node.py        # Python module
├── setup.py
└── README.md
```

## Usage

### 1. Complete SLAM Stack (Recommended)

Launch the complete stack with camera, image converter, and ORB-SLAM3:

```bash
ros2 launch sati_ros_camera_stack sati_orbslam_stack.launch.py
```

With custom parameters:
```bash
ros2 launch sati_ros_camera_stack sati_orbslam_stack.launch.py \
    camera:="/base/axi/pcie@1000120000/rp1/i2c@80000/ov9281@60" \
    format:="R8" \
    width:=640 \
    height:=480 \
    fps:=30 \
    vocabulary_path:="/home/orb/ORB_SLAM3/Vocabulary/ORBvoc.txt" \
    settings_path:="/root/colcon_ws/src/orb_slam3_ros2_wrapper/params/arducam_mono.yaml" \
    visualization:=false
```

### 2. Camera Stack Only

Launch just the camera stack with integrated image converter:

```bash
ros2 launch sati_ros_camera_stack sati_camera_stack.launch.py
```

With custom parameters:
```bash
ros2 launch sati_ros_camera_stack sati_camera_stack.launch.py \
    camera:="/base/axi/pcie@1000120000/rp1/i2c@80000/ov9281@60" \
    format:="R8" \
    width:=640 \
    height:=480 \
    fps:=30 \
    enable_converter:=true
```

### 3. Direct Node Execution

Run the image converter node directly:

```bash
ros2 run sati_ros_camera_stack image_converter_node
```

With custom parameters:
```bash
ros2 run sati_ros_camera_stack image_converter_node --ros-args \
    -p input_topic:="/my_camera/image_raw" \
    -p output_topic:="/my_camera/image_mono8"
```

## Topics

### Subscribed Topics
- `/camera/image_raw` (sensor_msgs/Image): Input camera images

### Published Topics
- `/camera/image_mono8` (sensor_msgs/Image): Converted mono8 images

## Parameters

### Image Converter Node Parameters
- `input_topic` (string, default: "/camera/image_raw"): Input image topic
- `output_topic` (string, default: "/camera/image_mono8"): Output mono8 image topic

### Launch File Parameters

#### Camera Parameters
- `camera` (string): Camera device path
- `format` (string, default: "R8"): Camera format (R8, YUYV, etc.)
- `width` (int, default: 640): Camera width
- `height` (int, default: 480): Camera height
- `fps` (int, default: 30): Camera FPS
- `frame_id` (string, default: "camera"): Camera frame ID

#### ORB-SLAM3 Parameters (sati_orbslam_stack.launch.py only)
- `vocabulary_path` (string): Path to ORB-SLAM3 vocabulary file
- `settings_path` (string): Path to ORB-SLAM3 settings file
- `image_topic_name` (string, default: "/camera/image_mono8"): Image topic for ORB-SLAM3
- `visualization` (bool, default: false): Enable ORB-SLAM3 visualization
- `enable_converter` (bool, default: true): Enable image converter

#### Frame Configuration Parameters (sati_orbslam_stack.launch.py only)
- `robot_base_frame` (string, default: "base_footprint"): Robot base frame ID
- `global_frame` (string, default: "map"): Global frame ID (usually map)
- `odom_frame` (string, default: "odom"): Odometry frame ID
- `publish_tf` (bool, default: true): Enable TF publishing from ORB-SLAM3

## Supported Image Formats

The image converter supports the following input formats:
- `mono8`: Pass-through (no conversion needed)
- `mono16`: Converted to mono8 by scaling down (divide by 256)
- `bgr8`: Converted to grayscale using OpenCV
- `bgra8`: Converted to BGR first, then to grayscale
- `rgb8`: Converted to grayscale using OpenCV

## Integration with ORB-SLAM3

This package is designed to work seamlessly with ORB-SLAM3, which requires mono8 format images.

### Option 1: Use the Complete Stack (Recommended)
Use the `sati_orbslam_stack.launch.py` which launches everything together:

```bash
ros2 launch sati_ros_camera_stack sati_orbslam_stack.launch.py
```

### Option 2: Manual Integration
Launch the camera stack separately and then ORB-SLAM3:

```bash
# Terminal 1: Launch camera stack
ros2 launch sati_ros_camera_stack sati_camera_stack.launch.py

# Terminal 2: Launch ORB-SLAM3
ros2 run orb_slam3_ros2_wrapper mono \
    /home/orb/ORB_SLAM3/Vocabulary/ORBvoc.txt \
    /root/colcon_ws/src/orb_slam3_ros2_wrapper/params/arducam_mono.yaml \
    --ros-args -p image_topic_name:="/camera/image_mono8" -p visualization:=false
```

## Frame Configuration

ORB-SLAM3 works with several coordinate frames that define the spatial relationships in your robot system:

### Frame Types

1. **`robot_base_frame`** (default: `base_footprint`): The main coordinate frame of your robot
2. **`global_frame`** (default: `map`): The global coordinate frame where the map is built
3. **`odom_frame`** (default: `odom`): The odometry frame for robot motion estimation
4. **`frame_id`** (camera frame, default: `camera`): The coordinate frame of the camera

### Setting Custom Frames

You can configure frames when launching the complete stack:

```bash
# Example: Custom robot frames
ros2 launch sati_ros_camera_stack sati_orbslam_stack.launch.py \
    robot_base_frame:="robot_base_link" \
    global_frame:="world" \
    odom_frame:="robot_odom" \
    frame_id:="camera_link"
```

```bash
# Example: Custom camera position (camera mounted 20cm forward, 5cm up)
ros2 launch sati_ros_camera_stack sati_orbslam_stack.launch.py \
    camera_x:=0.20 \
    camera_y:=0.0 \
    camera_z:=0.05
```

```bash
# Example: Multi-robot setup with namespaces
ros2 launch sati_ros_camera_stack sati_orbslam_stack.launch.py \
    robot_base_frame:="robot1/base_link" \
    global_frame:="world" \
    odom_frame:="robot1/odom" \
    frame_id:="robot1/camera"
```

### TF Tree Structure

When `publish_tf:=true` (default), the system publishes transforms in this hierarchy:
```
global_frame (orb_map)
└── odom_frame (odom)
    └── robot_base_frame (base_footprint)
        └── frame_id (camera)  [published by static_transform_publisher]
```

The `base_footprint -> camera` transform is published by a static transform publisher with configurable parameters:
- `camera_x`, `camera_y`, `camera_z`: Translation in meters (default: 0.16, 0.0, 0.0)
- `camera_roll`, `camera_pitch`, `camera_yaw`: Rotation in radians (default: 0.0, 0.0, 0.0)

To disable TF publishing (if you have another source):
```bash
ros2 launch sati_ros_camera_stack sati_orbslam_stack.launch.py publish_tf:=false
```

## Configuration

### Camera Transform Parameters

Configure the spatial relationship between your robot's base and camera:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `camera_x` | 0.16 | Camera X offset from base_footprint (meters) |
| `camera_y` | 0.0 | Camera Y offset from base_footprint (meters) |
| `camera_z` | 0.0 | Camera Z offset from base_footprint (meters) |
| `camera_roll` | 0.0 | Camera roll rotation (radians) |
| `camera_pitch` | 0.0 | Camera pitch rotation (radians) |
| `camera_yaw` | 0.0 | Camera yaw rotation (radians) |

**Important**: Measure these values from your actual robot hardware. The default assumes the camera is mounted 16cm forward of the robot's base center.

## Dependencies

- `rclpy`
- `sensor_msgs`
- `cv_bridge`
- `camera_ros`
- OpenCV (cv2)
- NumPy

## Building

```bash
cd ~/colcon_ws
colcon build --packages-select sati_ros_camera_stack
source install/setup.bash
```
