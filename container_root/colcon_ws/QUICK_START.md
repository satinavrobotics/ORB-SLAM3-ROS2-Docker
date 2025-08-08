# ORB-SLAM3 ROS 2 Monocular - Quick Start Guide

This is a condensed setup guide for experienced ROS users. For detailed instructions, see [SETUP_README.md](SETUP_README.md).

## Prerequisites Checklist

- [ ] Ubuntu 22.04 with ROS 2 Humble
- [ ] Monocular camera connected and working
- [ ] Basic ROS 2 development environment

## 5-Minute Setup

### 1. Install Dependencies

```bash
# Essential packages
sudo apt update
sudo apt install -y \
    ros-humble-desktop \
    python3-colcon-common-extensions \
    libopencv-dev \
    libeigen3-dev \
    libpcl-dev \
    libboost-all-dev \
    cmake \
    git

# Pangolin (for visualization)
cd /tmp
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin && mkdir build && cd build
cmake .. && make -j$(nproc) && sudo make install
```

### 2. Install ORB-SLAM3

```bash
cd ~
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git
cd ORB_SLAM3
chmod +x build.sh && ./build.sh
echo "export ORB_SLAM3_ROOT_DIR=~/ORB_SLAM3" >> ~/.bashrc
source ~/.bashrc
```

### 3. Build ROS 2 Workspace

```bash
mkdir -p ~/colcon_ws/src && cd ~/colcon_ws/src
# Clone your ORB-SLAM3 ROS 2 wrapper repository here
git clone <your-repo-url> orb_slam3_ros2_wrapper

cd ~/colcon_ws
rosdep install --from-paths src --ignore-src -r -y

# Build (use limited parallelism if you have <8GB RAM)
colcon build --symlink-install --parallel-workers 2

# For systems with limited memory (<4GB RAM):
# colcon build --symlink-install --parallel-workers 1

echo "source ~/colcon_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Quick Test

### 1. Start Camera

```bash
# USB camera example
ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:="/dev/video0"

# Or use your specific camera driver
```

### 2. Run ORB-SLAM3

```bash
# Terminal 1: Start SLAM (headless mode)
export LD_LIBRARY_PATH=/home/orb/ORB_SLAM3/lib:$LD_LIBRARY_PATH
ros2 run orb_slam3_ros2_wrapper mono \
    ~/ORB_SLAM3/Vocabulary/ORBvoc.txt \
    ~/colcon_ws/src/orb_slam3_ros2_wrapper/params/arducam_mono.yaml \
    --ros-args -p visualization:=false

# Terminal 2: Visualize (optional)
rviz2
```

### 3. Check Output

```bash
# Verify topics
ros2 topic list | grep -E "(pose|map)"

# Monitor pose
ros2 topic echo /robot_pose_slam --field pose.position
```

## Camera Calibration (Required)

If using a new camera, calibrate it first:

```bash
# Install calibration tools
sudo apt install ros-humble-camera-calibration

# Run calibration (print a checkerboard pattern)
ros2 run camera_calibration cameracalibrator \
    --size 8x6 --square 0.108 \
    image:=/camera/image_raw camera:=/camera

# Save results and update your camera config YAML
```

## Configuration Files

### Minimal Camera Config (`my_camera.yaml`)

```yaml
%YAML:1.0

Camera.type: "PinHole"

# Update these with your calibration results
Camera.fx: 615.0
Camera.fy: 615.0
Camera.cx: 320.0
Camera.cy: 240.0

Camera.k1: 0.0
Camera.k2: 0.0
Camera.p1: 0.0
Camera.p2: 0.0
Camera.k3: 0.0

Camera.width: 640
Camera.height: 480
Camera.fps: 30.0

# ORB Parameters (default values)
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

### ROS Parameters (`mono_params.yaml`)

```yaml
ORB_SLAM3_MONO_ROS2:
  ros__parameters:
    image_topic_name: "camera/image_raw"
    robot_base_frame: "base_footprint"
    global_frame: "map"
    publish_tf: true
    visualization: true
    do_loop_closing: true
```

## Common Issues & Quick Fixes

| Issue | Quick Fix |
|-------|-----------|
| "Build killed/memory error" | Use `colcon build --parallel-workers 1` |
| "No vocabulary file" | Check `~/ORB_SLAM3/Vocabulary/ORBvoc.txt` exists |
| "Tracking failed" | Move camera slowly, ensure good lighting |
| "Build errors" | Install missing dependencies, check ORB_SLAM3_ROOT_DIR |
| "No camera topic" | Verify camera driver is running: `ros2 topic list` |
| "Poor tracking" | Calibrate camera properly, adjust lighting |

## Launch File Template

Create `my_slam.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='orb_slam3_ros2_wrapper',
            executable='mono',
            arguments=[
                '/home/orb/ORB_SLAM3/Vocabulary/ORBvoc.txt',
                '/path/to/your/camera_config.yaml'
            ],
            parameters=[{
                'image_topic_name': '/camera/image_raw',
                'publish_tf': True,
                'visualization': True
            }]
        )
    ])
```

Run with: `ros2 launch your_package my_slam.launch.py`

## Next Steps

1. **Tune parameters** for your specific camera and environment
2. **Add IMU support** for monocular-inertial SLAM
3. **Integrate with navigation** stack for autonomous robots
4. **Record datasets** for testing and development

For detailed configuration and advanced features, see the full [SETUP_README.md](SETUP_README.md).

## Verification Commands

```bash
# Check installation
ros2 pkg executables orb_slam3_ros2_wrapper

# Monitor performance
ros2 topic hz /robot_pose_slam

# Debug tracking
ros2 topic echo /slam_info

# View TF tree
ros2 run tf2_tools view_frames
```

Success indicators:
- ✅ Pangolin window shows camera view and map points
- ✅ `/robot_pose_slam` publishes at ~20-30 Hz
- ✅ TF tree includes `map -> odom -> base_footprint`
- ✅ Map points appear in RViz2