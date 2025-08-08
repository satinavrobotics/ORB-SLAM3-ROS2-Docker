# ORB-SLAM3 ROS 2 Wrapper

A comprehensive ROS 2 wrapper for ORB-SLAM3 supporting both **monocular** and **RGB-D** cameras with full integration into the ROS 2 ecosystem.

## 🚀 Quick Links

- **[📋 Setup Guide](../../SETUP_README.md)** - Complete installation instructions
- **[⚡ Quick Start](../../QUICK_START.md)** - Get running in 5 minutes
- **[🔧 Troubleshooting](../../TROUBLESHOOTING.md)** - Common issues and solutions
- **[📖 This Guide](#usage)** - Basic usage and examples

## ✨ Features

- **Monocular SLAM** - Single camera visual SLAM
- **Monocular-Inertial SLAM** - Enhanced with IMU data
- **RGB-D SLAM** - Depth camera support
- **ROS 2 Native** - Full integration with ROS 2 ecosystem
- **Real-time Performance** - Optimized for live camera feeds
- **Loop Closure** - Automatic map correction and optimization
- **TF Integration** - Publishes transforms for navigation stack
- **Visualization** - Pangolin viewer and RViz2 support

## 📦 What's Included

### Executables
- `mono` - Monocular camera SLAM node
- `rgbd` - RGB-D camera SLAM node

### Launch Files
- `mono.launch.py` - Monocular SLAM launcher
- `rgbd.launch.py` - RGB-D SLAM launcher

### Configuration
- `arducam_mono.yaml` - Example monocular camera config
- `mono-ros-params.yaml` - ROS parameters for monocular node
- `rgbd-ros-params.yaml` - ROS parameters for RGB-D node

## 🎯 Supported Sensors

| Sensor Type | Node | IMU Support | Description |
|-------------|------|-------------|-------------|
| Monocular Camera | `mono` | ✅ Optional | Single camera visual SLAM |
| RGB-D Camera | `rgbd` | ✅ Optional | Depth-enhanced SLAM |
| Stereo Camera | `rgbd`* | ✅ Optional | Dual camera setup |

*Use RGB-D node with stereo configuration

This README explains how to run the ORB-SLAM3 ROS2 node and publish its topics for use with other ROS2 packages and visualization tools.

## 🔧 Quick Installation

```bash
# 1. Install dependencies
sudo apt install ros-humble-desktop libopencv-dev libeigen3-dev libpcl-dev

# 2. Install ORB-SLAM3
cd ~ && git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git
cd ORB_SLAM3 && chmod +x build.sh && ./build.sh

# 3. Build this package
mkdir -p ~/colcon_ws/src && cd ~/colcon_ws/src
git clone <this-repository> orb_slam3_ros2_wrapper
cd ~/colcon_ws && colcon build --symlink-install --parallel-workers 2

# Note: Use --parallel-workers 1 if you have limited RAM (<4GB)
```

For detailed setup instructions, see **[📋 Setup Guide](../../SETUP_README.md)**.

## Prerequisites
- ROS 2 Humble (recommended) or Foxy
- ORB-SLAM3 installed and built
- Camera driver publishing images (e.g., `/camera/image_raw`)
- Camera calibration YAML file (see `params/arducam_mono.yaml` for example)

## 📖 Usage

### 1. Start Camera Node(s)
Make sure your camera node is running and publishing to the correct topic:

```bash
ros2 run camera_ros camera_node --ros-args \
  -p format:="R8" \
  -p role:="video" \
  -p width:=640 -p height:=400 \
  -p fps:=20 \
  -p exposure_time:=8333 \
  -p camera:="/base/axi/pcie@1000120000/rp1/i2c@88000/ov9281@60" \ 
  -p frame_id:="camera"
```
Change i2c@88000 to i2c@80000 if connected to cam0

### 2. Configure ORB-SLAM3 Parameters
Edit your camera calibration YAML file (e.g., `params/arducam_mono.yaml`) to match your camera's intrinsic and distortion parameters.
(See Cameracalibration.md to get the calibration parameters)

### 3. Run ORB-SLAM3 ROS2 Node

### For Monocular Camera:
Launch the monocular ORB-SLAM3 node:

```bash
# With visualization (requires X11 display)
ros2 run orb_slam3_ros2_wrapper mono /home/orb/ORB_SLAM3/Vocabulary/ORBvoc.txt /root/colcon_ws/src/orb_slam3_ros2_wrapper/params/arducam_mono.yaml

# Headless mode (no visualization)
ros2 run orb_slam3_ros2_wrapper mono /home/orb/ORB_SLAM3/Vocabulary/ORBvoc.txt /root/colcon_ws/src/orb_slam3_ros2_wrapper/params/arducam_mono.yaml --ros-args -p visualization:=false

# To use a different image topic (e.g., mono8 converted images):
ros2 run orb_slam3_ros2_wrapper mono /home/orb/ORB_SLAM3/Vocabulary/ORBvoc.txt /root/colcon_ws/src/orb_slam3_ros2_wrapper/params/arducam_mono.yaml --ros-args -p image_topic_name:="/camera/image_mono8" -p visualization:=false
```

Or using the launch file:
```bash
# With visualization
ros2 launch orb_slam3_ros2_wrapper mono.launch.py

# Headless mode
ros2 launch orb_slam3_ros2_wrapper mono.launch.py visualization:=false
```

### For RGB-D Camera:
Launch the RGB-D ORB-SLAM3 node:

```bash
ros2 run orb_slam3_ros2_wrapper rgbd /home/orb/ORB_SLAM3/Vocabulary/ORBvoc.txt /path/to/rgbd_config.yaml
```

Or using the launch file:
```bash
ros2 launch orb_slam3_ros2_wrapper rgbd.launch.py
```

### Parameters:
- First argument: Path to ORB vocabulary file
- Second argument: Path to your camera calibration YAML file
- The node will automatically detect sensor type from the configuration file

### 4. Visualize Topics in RViz2
Start RViz2:
```bash
rviz2
```
Add the following displays:
- **Image**: `/camera/image_raw`
- **PointCloud2**: `/map_points` or `/visible_landmarks`
- **Pose**: `/robot_pose_slam`
- **Map Data**: `/map_data`

## Topics Published by Monocular Node

### Continuous Publications:
- `/map_data` (slam_msgs/MapData): Complete map data including keyframes and map points
- `/robot_pose_slam` (geometry_msgs/PoseStamped): Current robot pose in map frame
- `/slam_info` (slam_msgs/SlamInfo): SLAM status information

### Service-triggered Publications:
- `/map_points` (sensor_msgs/PointCloud2): All map points in current map
- `/visible_landmarks` (sensor_msgs/PointCloud2): Map points visible from a specific pose
- `/visible_landmarks_pose` (geometry_msgs/PoseStamped): Pose used for landmark visibility query

## Services Provided by Monocular Node

- `/get_map_data` (slam_msgs/srv/GetMap): Get complete map data
- `/get_all_map_points` (slam_msgs/srv/GetAllLandmarksInMap): Get all map points
- `/get_map_points_in_view` (slam_msgs/srv/GetLandmarksInView): Get landmarks visible from a pose
- `/reset_local_map` (std_srvs/srv/SetBool): Reset the local mapping

## Topics Subscribed by Monocular Node

- `/camera/image_raw` (sensor_msgs/Image): Camera images for SLAM (default topic)
- `/camera/image_mono8` (sensor_msgs/Image): Alternative mono8 topic (use with `-p image_topic_name:="/camera/image_mono8"`)
- `/imu` (sensor_msgs/Imu): IMU data for monocular-inertial SLAM (optional)
- `/odom` (nav_msgs/Odometry): Odometry data (if odometry_mode is enabled)

**Note**: If your camera publishes mono16 images, use the image converter to create a mono8 topic:
```bash
# Start image converter (converts mono16 to mono8)
python3 /root/image_converter.py &

# Then run ORB-SLAM3 with the converted topic
ros2 run orb_slam3_ros2_wrapper mono /home/orb/ORB_SLAM3/Vocabulary/ORBvoc.txt /root/colcon_ws/src/orb_slam3_ros2_wrapper/params/arducam_mono.yaml --ros-args -p image_topic_name:="/camera/image_mono8" -p visualization:=false
```

## 5. Interact with Landmarks Service
You can call the `orb_slam3/get_landmarks_in_view` service to get visible landmarks for a given pose:
```bash
ros2 service call /orb_slam3/get_landmarks_in_view slam_msgs/srv/GetLandmarksInView "{pose: ...}"
```

## 6. Troubleshooting
- Ensure all topics are being published (use `ros2 topic list` and `ros2 topic echo <topic>`)
- Check camera calibration matches your hardware
- Review node logs for errors
- **"Not initialized" messages**: This is normal during startup. ORB-SLAM3 needs:
  - Sufficient lighting and textured surfaces (avoid blank walls)
  - Camera movement to detect features and establish tracking
  - At least 100-200 ORB features detected per frame
  - Proper camera calibration parameters
- **Image Encoding Issues**: If ORB-SLAM3 shows "Not initialized" with mono16 images, use the image converter:
  ```bash
  # Check current encoding
  ros2 topic echo /camera/image_raw --field encoding
  
  # If mono16, start converter and use converted topic
  python3 /root/image_converter.py &
  ros2 run orb_slam3_ros2_wrapper mono [args] --ros-args -p image_topic_name:="/camera/image_mono8"
  ```

## 7. Example Workflow
1. **Start camera node(s)**
   ```bash
   ros2 run camera_ros camera_node --ros-args -p camera:="/base/axi/pcie@1000120000/rp1/i2c@80000/ov9281@60" -p width:=640 -p height:=400
   ```

2. **Start image converter** (if using mono16 camera)
   ```bash
   python3 /root/image_converter.py &
   ```

3. **Start ORB-SLAM3 node**
   ```bash
   ros2 run orb_slam3_ros2_wrapper mono /home/orb/ORB_SLAM3/Vocabulary/ORBvoc.txt /root/colcon_ws/src/orb_slam3_ros2_wrapper/params/arducam_mono.yaml --ros-args -p image_topic_name:="/camera/image_mono8" -p visualization:=false
   ```

4. **Initialize SLAM tracking**
   - Move the camera slowly with sufficient lighting
   - Point camera at textured surfaces (avoid blank walls)
   - ORB-SLAM3 will show "Not initialized" until it detects enough features
   - Once initialized, you'll see tracking status change to "OK"

5. **Visualize in RViz2** (optional)
   ```bash
   rviz2
   ```

6. **Use services and topics as needed**

---

For more details, see the source code and parameter files in this repository.