# ORB-SLAM3 ROS 2 Wrapper Setup Guide

This guide provides step-by-step instructions for setting up the ORB-SLAM3 ROS 2 wrapper with monocular camera support.

## Prerequisites

### System Requirements
- Ubuntu 22.04 LTS (recommended)
- ROS 2 Humble Hawksbill
- OpenCV 4.x
- Eigen3
- Pangolin
- PCL (Point Cloud Library)

### Hardware Requirements
- Camera (monocular or RGB-D)
- IMU sensor (optional, for monocular-inertial SLAM)
- Sufficient computational resources (recommended: 4+ CPU cores, 8GB+ RAM)

## Installation Steps

### 1. Install ROS 2 Humble

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install ROS 2 Humble
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt update
sudo apt install ros-humble-desktop python3-argcomplete -y

# Source ROS 2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. Install Dependencies

```bash
# Install build tools
sudo apt install python3-colcon-common-extensions python3-rosdep -y

# Initialize rosdep
sudo rosdep init
rosdep update

# Install OpenCV
sudo apt install libopencv-dev python3-opencv -y

# Install Eigen3
sudo apt install libeigen3-dev -y

# Install PCL
sudo apt install libpcl-dev -y

# Install additional dependencies
sudo apt install libboost-all-dev libssl-dev -y
```

### 3. Install Pangolin

```bash
# Install Pangolin dependencies
sudo apt install libgl1-mesa-dev libglew-dev cmake libpython3-dev -y

# Clone and build Pangolin
cd ~
git clone --recursive https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

### 4. Install ORB-SLAM3

```bash
# Clone ORB-SLAM3
cd ~
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git
cd ORB_SLAM3

# Install DBoW2 and g2o (included in ORB-SLAM3)
chmod +x build.sh
./build.sh

# Set environment variable
echo "export ORB_SLAM3_ROOT_DIR=~/ORB_SLAM3" >> ~/.bashrc
source ~/.bashrc
```

### 5. Create ROS 2 Workspace

```bash
# Create workspace
mkdir -p ~/colcon_ws/src
cd ~/colcon_ws/src

# Clone the ORB-SLAM3 ROS 2 wrapper (your repository)
git clone <your-repository-url> orb_slam3_ros2_wrapper

# Clone additional message packages if needed
git clone https://github.com/your-repo/slam_msgs.git  # If not included
```

### 6. Install ROS 2 Dependencies

```bash
cd ~/colcon_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 7. Build the Workspace

```bash
cd ~/colcon_ws

# For systems with 8GB+ RAM
colcon build --symlink-install

# For systems with 4-8GB RAM (recommended)
colcon build --symlink-install --parallel-workers 2

# For systems with <4GB RAM or if build fails with memory errors
colcon build --symlink-install --parallel-workers 1

# Source the workspace
echo "source ~/colcon_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**Note:** The monocular node implementation requires significant memory during compilation. If you encounter "Killed (signal 9)" errors, reduce parallelism or add swap space.

## Camera Setup

### 1. Camera Calibration

Before using ORB-SLAM3, you need to calibrate your camera:

```bash
# Install camera calibration tools
sudo apt install ros-humble-camera-calibration -y

# Run calibration (replace with your camera topic)
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.108 image:=/camera/image_raw camera:=/camera
```

### 2. Create Camera Configuration File

Create a YAML file with your camera parameters (example: `params/my_camera_mono.yaml`):

```yaml
%YAML:1.0

#--------------------------------------------------------------------------------------------
# Camera Parameters
#--------------------------------------------------------------------------------------------
Camera.type: "PinHole"

# Camera calibration parameters (from calibration)
Camera.fx: 615.123
Camera.fy: 615.456
Camera.cx: 320.0
Camera.cy: 240.0

# Distortion parameters
Camera.k1: 0.1234
Camera.k2: -0.5678
Camera.p1: 0.0012
Camera.p2: -0.0034
Camera.k3: 0.9876

Camera.width: 640
Camera.height: 480
Camera.fps: 30.0

# ORB Parameters
ORBextractor.nFeatures: 1250
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels: 8
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7

# Viewer Parameters
Viewer.KeyFrameSize: 0.05
Viewer.KeyFrameLineWidth: 1
Viewer.GraphLineWidth: 0.9
Viewer.PointSize: 2
Viewer.CameraSize: 0.08
Viewer.CameraLineWidth: 3
Viewer.ViewpointX: 0
Viewer.ViewpointY: -0.7
Viewer.ViewpointZ: -1.8
Viewer.ViewpointF: 500
```

## Running the System

### 1. Start Your Camera Node

```bash
# Example for a USB camera
ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:="/dev/video0"

# Or for the camera_ros package
ros2 run camera_ros camera_node --ros-args -p format:="R8" -p width:=640 -p height:=480
```

### 2. Launch ORB-SLAM3 Monocular Node

```bash
# Method 1: Direct execution
ros2 run orb_slam3_ros2_wrapper mono ~/ORB_SLAM3/Vocabulary/ORBvoc.txt ~/colcon_ws/src/orb_slam3_ros2_wrapper/params/my_camera_mono.yaml

# Method 2: Using launch file
ros2 launch orb_slam3_ros2_wrapper mono.launch.py
```

### 3. Visualize in RViz2

```bash
# Start RViz2
rviz2

# Add displays:
# - Image: /camera/image_raw
# - PointCloud2: /map_points
# - PoseStamped: /robot_pose_slam
# - Path: /trajectory (if available)
```

## Configuration

### Parameter Files

Edit `params/mono-ros-params.yaml` to configure:

- **Topics**: Camera and IMU topic names
- **Frames**: TF frame IDs
- **SLAM settings**: Loop closing, visualization, etc.
- **Publishing rates**: Map data frequency

### Key Parameters

```yaml
ORB_SLAM3_MONO_ROS2:
  ros__parameters:
    image_topic_name: "camera/image_raw"  # Your camera topic
    imu_topic_name: "imu"                 # Your IMU topic (optional)
    robot_base_frame: "base_footprint"    # Robot base frame
    global_frame: "map"                   # Global map frame
    publish_tf: true                      # Publish TF transforms
    do_loop_closing: true                 # Enable loop closure
    visualization: true                   # Enable Pangolin viewer
```

## Troubleshooting

### Common Issues

1. **"No vocabulary file found"**
   - Ensure ORB_SLAM3 is properly installed
   - Check vocabulary file path: `~/ORB_SLAM3/Vocabulary/ORBvoc.txt`

2. **"Camera calibration file not found"**
   - Verify the path to your camera configuration YAML
   - Ensure the file format matches ORB-SLAM3 requirements

3. **"Tracking failed: Not initialized"**
   - Move the camera to provide sufficient visual features
   - Ensure good lighting conditions
   - Check camera calibration parameters

4. **Build errors**
   - Verify all dependencies are installed
   - Check ORB_SLAM3_ROOT_DIR environment variable
   - Ensure Pangolin is properly installed

### Performance Tips

- Use a powerful CPU (4+ cores recommended)
- Ensure good lighting for the camera
- Avoid rapid camera movements during initialization
- Use appropriate ORB feature extraction parameters for your scene

## Testing

### Verify Installation

```bash
# Check if nodes are available
ros2 pkg executables orb_slam3_ros2_wrapper

# Test with a rosbag (if available)
ros2 bag play your_test_bag.bag

# Check topics
ros2 topic list
ros2 topic echo /robot_pose_slam
```

### Expected Output

When working correctly, you should see:
- Camera tracking status in terminal
- 3D visualization in Pangolin viewer
- Published pose and map data on ROS topics
- TF transforms between map and robot frames

## Support

For issues and questions:
1. Check the troubleshooting section above
2. Verify your camera calibration
3. Ensure all dependencies are properly installed
4. Check ROS 2 topic connections and data flow

## Advanced Configuration

### Monocular-Inertial SLAM

To use IMU data with monocular SLAM:

1. **Add IMU parameters to your camera config file:**

```yaml
# IMU noise parameters
IMU.NoiseGyro: 1.6968e-04    # rad/s^0.5
IMU.NoiseAcc: 2.0000e-3      # m/s^1.5
IMU.GyroWalk: 1.9393e-05     # rad/s^1.5
IMU.AccWalk: 3.0000e-3       # m/s^2.5
IMU.Frequency: 200           # Hz

# Transformation from camera to IMU
Tbc: !!opencv-matrix
   rows: 4
   cols: 4
   dt: f
   data: [1.0, 0.0, 0.0, 0.0,
          0.0, 1.0, 0.0, 0.0,
          0.0, 0.0, 1.0, 0.0,
          0.0, 0.0, 0.0, 1.0]
```

2. **Enable IMU in parameters:**

```yaml
ORB_SLAM3_MONO_ROS2:
  ros__parameters:
    imu_topic_name: "/imu/data"  # Your IMU topic
```

### Multi-Camera Setup

For stereo or multi-camera setups, use the RGBD node with appropriate configuration.

### Custom Launch Files

Create custom launch files for your specific setup:

```python
#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='orb_slam3_ros2_wrapper',
            executable='mono',
            name='orb_slam3_mono',
            output='screen',
            arguments=[
                '/home/orb/ORB_SLAM3/Vocabulary/ORBvoc.txt',
                '/path/to/your/camera_config.yaml'
            ],
            parameters=[{
                'image_topic_name': '/your_camera/image_raw',
                'imu_topic_name': '/your_imu/data',
                'publish_tf': True,
                'visualization': True
            }]
        )
    ])
```

## Integration Examples

### With Navigation Stack

```bash
# Install Nav2
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup -y

# Use ORB-SLAM3 pose for localization
# Configure Nav2 to use /robot_pose_slam topic
```

### With MoveIt

```bash
# Install MoveIt
sudo apt install ros-humble-moveit -y

# Use ORB-SLAM3 for environment mapping
# Subscribe to /map_points for obstacle detection
```

### Recording Data

```bash
# Record a bag for testing
ros2 bag record /camera/image_raw /imu/data -o test_slam_data

# Play back for testing
ros2 bag play test_slam_data
```

## Performance Optimization

### System Tuning

```bash
# Increase CPU performance
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor

# Increase memory limits
ulimit -c unlimited
```

### Camera Settings

- **Resolution**: 640x480 or 1280x720 (balance quality vs performance)
- **Frame Rate**: 20-30 FPS (higher rates need more CPU)
- **Exposure**: Fixed exposure for consistent lighting
- **Focus**: Fixed focus to avoid autofocus delays

### ORB-SLAM3 Tuning

Adjust these parameters in your camera config:

```yaml
# Reduce features for better performance
ORBextractor.nFeatures: 800

# Adjust scale factor for your environment
ORBextractor.scaleFactor: 1.2

# Tune FAST threshold for feature detection
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7
```
