# ORB-SLAM3 ROS2 Wrapper - Quick Start Guide

This README explains how to run the ORB-SLAM3 ROS2 node and publish its topics for use with other ROS2 packages and visualization tools.

## Prerequisites
- ROS2 (e.g., Humble, Foxy, etc.) installed and sourced
- ORB-SLAM3 ROS2 wrapper built and installed
- Camera node(s) running and publishing images (e.g., `/camera/image_raw`)
- Camera calibration YAML file (see `params/arducam_mono.yaml` for example)

## 1. Start Camera Node(s)
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

## 2. Configure ORB-SLAM3 Parameters
Edit your camera calibration YAML file (e.g., `params/arducam_mono.yaml`) to match your camera's intrinsic and distortion parameters.
(Run calibrator.py)

## 3. Run ORB-SLAM3 ROS2 Node
Launch the ORB-SLAM3 node with the correct parameters:

```bash
ros2 run orb_slam3_ros2_wrapper orb_slam3_node --ros-args \
  -p voc_file:=/path/to/ORBvoc.txt \
  -p camera_config:=/path/to/params/arducam_mono.yaml \
  -p sensor:="MONO" \
  -p image_topic:=/camera/image_raw \
  -p publish_map_points:=true \
  -p publish_pose:=true
```

- `voc_file`: Path to ORB vocabulary file
- `camera_config`: Path to your camera calibration YAML
- `sensor`: Sensor type (`MONO`, `STEREO`, `RGBD`)
- `image_topic`: Topic where camera images are published
- `publish_map_points`: Publish map points as PointCloud2
- `publish_pose`: Publish camera pose as PoseStamped

## 4. Visualize Topics in RViz2
Start RViz2:
```bash
rviz2
```
Add the following displays:
- **Image**: `/camera/image_raw`
- **PointCloud2**: `/map_points` or `/landmarks_in_view`
- **Pose**: `/orb_slam3/pose`

## 5. Interact with Landmarks Service
You can call the `orb_slam3/get_landmarks_in_view` service to get visible landmarks for a given pose:
```bash
ros2 service call /orb_slam3/get_landmarks_in_view slam_msgs/srv/GetLandmarksInView "{pose: ...}"
```

## 6. Troubleshooting
- Ensure all topics are being published (use `ros2 topic list` and `ros2 topic echo <topic>`)
- Check camera calibration matches your hardware
- Review node logs for errors

## 7. Example Workflow
1. Start camera node(s)
2. Start ORB-SLAM3 node
3. Visualize in RViz2
4. Use services and topics as needed

---

For more details, see the source code and parameter files in this repository.
