# ORB-SLAM3 ROS 2 Troubleshooting Guide

This guide covers common issues and their solutions when using the ORB-SLAM3 ROS 2 wrapper with monocular cameras.

## Build Issues

### 1. Memory Issues During Compilation

**Error:**
```
c++: fatal error: Killed (signal 9)
compilation terminated.
make[2]: *** [CMakeFiles/mono.dir/src/monocular/monocular-slam-node.cpp.o] Error 1
```

**Cause:**
The new monocular node implementation increases memory usage during compilation due to:
- Additional source files (monocular-slam-node.cpp, mono.cpp)
- Template instantiations in ORB-SLAM3 interface
- Parallel compilation overwhelming available RAM

**Solutions:**

1. **Reduce parallel jobs:**
   ```bash
   # Instead of default parallel build
   colcon build --symlink-install

   # Use limited parallelism
   colcon build --symlink-install --parallel-workers 1

   # Or for systems with 4GB+ RAM
   colcon build --symlink-install --parallel-workers 2
   ```

2. **Build specific packages only:**
   ```bash
   # Build just the ORB-SLAM3 wrapper
   colcon build --symlink-install --packages-select orb_slam3_ros2_wrapper

   # Skip other packages if not needed
   colcon build --symlink-install --packages-skip <other_packages>
   ```

3. **Increase swap space:**
   ```bash
   # Check current swap
   free -h

   # Add temporary swap file (4GB)
   sudo fallocate -l 4G /swapfile
   sudo chmod 600 /swapfile
   sudo mkswap /swapfile
   sudo swapon /swapfile

   # Build with swap enabled
   colcon build --symlink-install --parallel-workers 1

   # Remove swap after build (optional)
   sudo swapoff /swapfile
   sudo rm /swapfile
   ```

4. **Use make instead of ninja:**
   ```bash
   # Force make build system (uses less memory)
   colcon build --symlink-install --cmake-args -G "Unix Makefiles"
   ```

5. **Build in stages:**
   ```bash
   # Build dependencies first
   colcon build --symlink-install --packages-up-to slam_msgs

   # Then build the main package
   colcon build --symlink-install --packages-select orb_slam3_ros2_wrapper
   ```

**Memory Requirements:**
- **Minimum**: 4GB RAM + 2GB swap for single-threaded build
- **Recommended**: 8GB RAM for parallel build
- **With limited RAM**: Use `--parallel-workers 1` and ensure 2GB+ swap

### 2. Runtime Library Loading Issues

**Error:**
```
error while loading shared libraries: libORB_SLAM3.so: cannot open shared object file: No such file or directory
```

**Cause:**
The ORB-SLAM3 library path is not in the system's library search path.

**Solutions:**

1. **Set library path temporarily:**
   ```bash
   export LD_LIBRARY_PATH=/home/orb/ORB_SLAM3/lib:$LD_LIBRARY_PATH
   source install/setup.bash
   ros2 run orb_slam3_ros2_wrapper mono [args...]
   ```

2. **Set library path permanently:**
   ```bash
   echo "export LD_LIBRARY_PATH=/home/orb/ORB_SLAM3/lib:\$LD_LIBRARY_PATH" >> ~/.bashrc
   source ~/.bashrc
   ```

3. **Add to system library path:**
   ```bash
   echo "/home/orb/ORB_SLAM3/lib" | sudo tee /etc/ld.so.conf.d/orbslam3.conf
   sudo ldconfig
   ```

4. **Verify library location:**
   ```bash
   find /home/orb/ORB_SLAM3 -name "libORB_SLAM3.so" -type f
   ldd install/orb_slam3_ros2_wrapper/lib/orb_slam3_ros2_wrapper/mono
   ```

### 3. CMake Cannot Find ORB_SLAM3

**Error:**
```
CMake Error: Could not find ORB_SLAM3
```

**Solutions:**
```bash
# Set environment variable
export ORB_SLAM3_ROOT_DIR=~/ORB_SLAM3
echo "export ORB_SLAM3_ROOT_DIR=~/ORB_SLAM3" >> ~/.bashrc

# Verify ORB-SLAM3 installation
ls ~/ORB_SLAM3/lib/libORB_SLAM3.so

# Rebuild if necessary
cd ~/ORB_SLAM3
./build.sh
```

### 2. Pangolin Not Found

**Error:**
```
Could not find Pangolin
```

**Solutions:**
```bash
# Install Pangolin dependencies
sudo apt install libgl1-mesa-dev libglew-dev cmake

# Reinstall Pangolin
cd /tmp
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin && mkdir build && cd build
cmake .. && make -j$(nproc) && sudo make install
sudo ldconfig
```

### 3. PCL Version Conflicts

**Error:**
```
PCL version mismatch
```

**Solutions:**
```bash
# Install correct PCL version
sudo apt remove libpcl-dev
sudo apt install libpcl-dev

# Clear build cache
cd ~/colcon_ws
rm -rf build/ install/
colcon build --symlink-install
```

## Runtime Issues

### 1. Tracking Initialization Fails

**Symptoms:**
- "ORB-SLAM failed: Not initialized" messages
- No map points visible
- Pangolin window shows only camera feed

**Solutions:**

1. **Improve visual conditions:**
   ```bash
   # Ensure good lighting
   # Move camera slowly with rich visual features
   # Avoid plain walls or repetitive patterns
   ```

2. **Check camera calibration:**
   ```bash
   # Verify camera parameters in YAML file
   # Recalibrate if necessary
   ros2 run camera_calibration cameracalibrator \
       --size 8x6 --square 0.108 \
       image:=/camera/image_raw camera:=/camera
   ```

3. **Adjust ORB parameters:**
   ```yaml
   # In camera config YAML, try:
   ORBextractor.nFeatures: 2000  # Increase features
   ORBextractor.iniThFAST: 15    # Lower threshold
   ORBextractor.minThFAST: 5     # Lower threshold
   ```

### 2. Tracking Lost Frequently

**Symptoms:**
- "ORB-SLAM failed: Tracking LOST" messages
- Intermittent pose publishing
- Map points disappearing

**Solutions:**

1. **Reduce camera motion speed:**
   - Move camera more slowly
   - Avoid rapid rotations
   - Ensure smooth motion

2. **Improve environment:**
   - Add more visual features to scene
   - Ensure consistent lighting
   - Avoid motion blur

3. **Tune tracking parameters:**
   ```yaml
   # More conservative settings
   ORBextractor.nFeatures: 1500
   ORBextractor.scaleFactor: 1.15
   ORBextractor.nLevels: 10
   ```

### 3. No Camera Images Received

**Symptoms:**
- "No images yet" error
- Empty Pangolin window
- No image topics

**Diagnostic Commands:**
```bash
# Check camera topics
ros2 topic list | grep image

# Test camera stream
ros2 topic echo /camera/image_raw --field header.stamp

# Check camera driver
ros2 node list | grep camera
```

**Solutions:**

1. **Verify camera driver:**
   ```bash
   # For USB cameras
   ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:="/dev/video0"
   
   # Check device permissions
   ls -l /dev/video*
   sudo chmod 666 /dev/video0
   ```

2. **Check topic names:**
   ```bash
   # Update topic name in parameters
   ros2 param set /orb_slam3_mono image_topic_name "/your_camera/image_raw"
   ```

3. **Verify image format:**
   ```bash
   # Check image encoding
   ros2 topic echo /camera/image_raw --field encoding
   # Should be: mono8, bgr8, or rgb8
   ```

### 4. Pangolin X11 Display Error

**Symptoms:**
- "Pangolin X11: Failed to open X display" error
- Node terminates after initialization
- Running in headless environment

**Solutions:**

1. **Disable visualization (recommended for headless):**
   ```bash
   # Method 1: Using parameter override
   ros2 run orb_slam3_ros2_wrapper mono [args...] --ros-args -p visualization:=false

   # Method 2: Using launch file
   ros2 launch orb_slam3_ros2_wrapper mono.launch.py visualization:=false

   # Method 3: Edit parameter file
   # In params/mono-ros-params.yaml, set:
   visualization: false
   ```

2. **Enable X11 forwarding (if using SSH):**
   ```bash
   ssh -X username@hostname
   # or
   ssh -Y username@hostname
   ```

3. **Use virtual display:**
   ```bash
   sudo apt install xvfb
   xvfb-run -a ros2 run orb_slam3_ros2_wrapper mono [args...]
   ```

**Note:** Disabling visualization doesn't affect SLAM functionality - the node will still track, build maps, and publish poses.

### 5. Poor Tracking Performance

**Symptoms:**
- Low frame rate
- Delayed pose updates
- High CPU usage

**Solutions:**

1. **Optimize camera settings:**
   ```bash
   # Reduce resolution
   # Lower frame rate to 20-25 FPS
   # Use fixed exposure and focus
   ```

2. **Tune ORB parameters:**
   ```yaml
   # Reduce computational load
   ORBextractor.nFeatures: 800
   ORBextractor.nLevels: 6
   ```

3. **System optimization:**
   ```bash
   # Set CPU governor to performance
   echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
   
   # Increase process priority
   sudo nice -n -10 ros2 run orb_slam3_ros2_wrapper mono ...
   ```

## Configuration Issues

### 1. Incorrect Camera Parameters

**Symptoms:**
- Poor tracking quality
- Incorrect scale estimation
- Distorted visualization

**Solutions:**

1. **Recalibrate camera:**
   ```bash
   ros2 run camera_calibration cameracalibrator \
       --size 9x6 --square 0.025 \
       image:=/camera/image_raw camera:=/camera
   ```

2. **Verify calibration quality:**
   - Reprojection error should be < 0.5 pixels
   - Use sufficient calibration images (>20)
   - Cover entire image area during calibration

3. **Check parameter format:**
   ```yaml
   # Ensure correct YAML format
   Camera.fx: 615.123  # Not "615.123"
   Camera.fy: 615.456
   Camera.cx: 320.0
   Camera.cy: 240.0
   ```

### 2. TF Frame Issues

**Symptoms:**
- No TF transforms published
- Incorrect robot pose
- Navigation integration problems

**Diagnostic Commands:**
```bash
# Check TF tree
ros2 run tf2_tools view_frames

# Monitor transforms
ros2 run tf2_ros tf2_echo map base_footprint

# List frames
ros2 run tf2_ros tf2_monitor
```

**Solutions:**

1. **Verify frame names:**
   ```yaml
   # In parameters file
   robot_base_frame: "base_footprint"  # Match your robot
   global_frame: "map"
   odom_frame: "odom"
   ```

2. **Enable TF publishing:**
   ```yaml
   publish_tf: true
   ```

3. **Check frame relationships:**
   ```bash
   # Should see: map -> odom -> base_footprint
   ros2 run tf2_tools view_frames
   ```

## Performance Optimization

### 1. Memory Issues

**Symptoms:**
- System slowdown
- Out of memory errors
- Process crashes

**Solutions:**

1. **Monitor memory usage:**
   ```bash
   # Check memory consumption
   top -p $(pgrep -f orb_slam3)
   
   # Monitor ROS node memory
   ros2 node info /orb_slam3_mono
   ```

2. **Optimize map size:**
   ```yaml
   # Limit map points
   ORBextractor.nFeatures: 1000
   
   # Enable loop closing to merge maps
   do_loop_closing: true
   ```

3. **System tuning:**
   ```bash
   # Increase swap if needed
   sudo swapon --show
   
   # Clear system cache
   sudo sync && sudo sysctl vm.drop_caches=3
   ```

### 2. Network Issues (Multi-machine Setup)

**Symptoms:**
- Delayed image transmission
- Topic communication failures
- Inconsistent performance

**Solutions:**

1. **Check network configuration:**
   ```bash
   # Verify ROS_DOMAIN_ID
   echo $ROS_DOMAIN_ID
   
   # Test network connectivity
   ros2 topic list
   ros2 topic hz /camera/image_raw
   ```

2. **Optimize QoS settings:**
   ```python
   # In launch file or node
   qos_profile = rclpy.qos.QoSProfile(
       reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
       history=rclpy.qos.HistoryPolicy.KEEP_LAST,
       depth=1
   )
   ```

## Debugging Tools

### Useful Commands

```bash
# Monitor all SLAM topics
ros2 topic list | grep -E "(slam|pose|map)"

# Check node status
ros2 node info /orb_slam3_mono

# Monitor computational load
htop -p $(pgrep -f orb_slam3)

# Record data for analysis
ros2 bag record /camera/image_raw /robot_pose_slam

# Analyze bag files
ros2 bag info your_bag.bag
```

### Log Analysis

```bash
# Enable debug logging
ros2 run orb_slam3_ros2_wrapper mono ... --ros-args --log-level DEBUG

# Check system logs
journalctl -f | grep orb_slam

# Monitor ROS logs
ros2 run rqt_console rqt_console
```

## Getting Help

### Information to Provide

When seeking help, include:

1. **System information:**
   ```bash
   lsb_release -a
   ros2 --version
   ```

2. **Build information:**
   ```bash
   colcon list
   echo $ORB_SLAM3_ROOT_DIR
   ```

3. **Runtime logs:**
   ```bash
   ros2 topic list
   ros2 node list
   ros2 param list /orb_slam3_mono
   ```

4. **Camera information:**
   ```bash
   ros2 topic info /camera/image_raw
   ros2 topic hz /camera/image_raw
   ```

### 6. Segmentation Fault During Vocabulary Loading

**Error:**
```
Loading ORB Vocabulary. This could take a while...
[ros2run]: Segmentation fault
```

**Cause:**
Missing ORB vocabulary file (ORBvoc.txt) which is required for ORB-SLAM3 initialization.

**Solutions:**

1. **Download vocabulary file:**
   ```bash
   # Clone ORB-SLAM3 repository to get vocabulary
   cd /tmp
   git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git
   cd /tmp/ORB_SLAM3/Vocabulary
   tar -xzf ORBvoc.txt.tar.gz
   
   # Copy to your wrapper directory
   cp ORBvoc.txt /root/colcon_ws/src/orb_slam3_ros2_wrapper/
   ```

2. **Verify file exists and size:**
   ```bash
   ls -la /root/colcon_ws/src/orb_slam3_ros2_wrapper/ORBvoc.txt
   # Should be ~145MB in size
   ```

3. **Update command with correct path:**
   ```bash
   ros2 run orb_slam3_ros2_wrapper mono \
     /root/colcon_ws/src/orb_slam3_ros2_wrapper/ORBvoc.txt \
     /root/colcon_ws/src/orb_slam3_ros2_wrapper/params/arducam_mono.yaml \
     --ros-args -p visualization:=false
   ```

### Common Log Messages

| Message | Meaning | Action |
|---------|---------|--------|
| "Not initialized" | SLAM not started | Move camera, improve lighting |
| "Tracking LOST" | Lost visual tracking | Slow down, improve features |
| "cv_bridge exception" | Image format issue | Check image encoding |
| "Waiting for merge" | Loop closure active | Normal, wait for completion |
| "No images yet" | No camera data | Check camera driver and topics |
| "Segmentation fault" | Missing vocabulary file | Download ORBvoc.txt file |
