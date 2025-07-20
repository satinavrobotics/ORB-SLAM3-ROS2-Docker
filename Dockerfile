### Dockerfile for ARM64-compatible ROS 2 Humble Desktop Full
# Switched to official multi-arch `ros:` image instead of `osrf/ros` to support ARM64
FROM ros:humble-perception-jammy 
ARG USE_CI

# Ensure package index is up to date
RUN apt-get update

# Install essential tools and libraries
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get install -y \
    gnupg2 curl lsb-base vim wget python3-pip libpng16-16 libjpeg-turbo8 libtiff5 \
    cmake build-essential git unzip pkg-config python3-dev python3-numpy \
    libgl1-mesa-dev libglew-dev libeigen3-dev apt-transport-https \
    ca-certificates software-properties-common \
    python2-dev libavcodec-dev libavformat-dev libswscale-dev \
    libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev libgtk-3-dev

# Build OpenCV 4.4.0
RUN cd /tmp && \
    git clone https://github.com/opencv/opencv.git && \
    cd opencv && git checkout 4.4.0 && mkdir build && cd build && \
    cmake -D CMAKE_BUILD_TYPE=Release \
          -D BUILD_EXAMPLES=OFF \
          -D BUILD_DOCS=OFF \
          -D BUILD_PERF_TESTS=OFF \
          -D BUILD_TESTS=OFF \
          -D CMAKE_INSTALL_PREFIX=/usr/local .. && \
    make -j$(nproc) && make install && \
    rm -rf /tmp/opencv

# Build Pangolin v0.9.1
RUN cd /tmp && \
    git clone https://github.com/stevenlovegrove/Pangolin.git && \
    cd Pangolin && git checkout v0.9.1 && mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release \
          -DCMAKE_CXX_FLAGS=-std=c++14 \
          -DCMAKE_INSTALL_PREFIX=/usr/local .. && \
    make -j$(nproc) && make install && \
    rm -rf /tmp/Pangolin

# Install VSCode via helper script
COPY ./container_root/shell_scripts/vscode_install.sh /root/
RUN chmod +x /root/vscode_install.sh && \
    /root/vscode_install.sh && rm -f /root/vscode_install.sh

# Install additional ROS 2 packages
RUN apt-get update && apt-get install -y \
    ros-humble-pcl-ros tmux x11-apps nano \
    gdb gdbserver ros-humble-rmw-cyclonedds-cpp \
    ros-humble-cv-bridge ros-humble-image-transport \
    ros-humble-image-common ros-humble-vision-opencv ros-humble-nav2-common

# Copy SLAM project sources
COPY ORB_SLAM3 /home/orb/ORB_SLAM3
COPY orb_slam3_ros2_wrapper /root/colcon_ws/src/orb_slam3_ros2_wrapper
COPY orb_slam3_map_generator /root/colcon_ws/src/orb_slam3_map_generator
COPY slam_msgs /root/colcon_ws/src/slam_msgs

# Build ORB-SLAM3 and ROS2 workspace if in CI
RUN if [ "$USE_CI" = "true" ]; then \
      . /opt/ros/humble/setup.sh && \
      cd /home/orb/ORB_SLAM3 && mkdir -p build && ./build.sh && \
      . /opt/ros/humble/setup.sh && \
      cd /root/colcon_ws/ && colcon build --symlink-install; \
    fi

RUN sudo apt-get install -y clang meson ninja-build pkg-config libyaml-dev python3-yaml python3-ply python3-jinja2 openssl
RUN sudo apt-get install -y libdw-dev libunwind-dev libudev-dev libudev-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libpython3-dev pybind11-dev libevent-dev libtiff-dev qt6-base-dev qt6-tools-dev-tools liblttng-ust-dev python3-jinja2 lttng-tools libexif-dev libjpeg-dev pybind11-dev libevent-dev libgtest-dev abi-compliance-checker

# libcamera
RUN apt-get remove -y meson && pip3 install meson && export PATH=$HOME/.local/bin:$PATH
RUN cd /tmp && git clone https://github.com/raspberrypi/libcamera.git  && \
    cd /tmp/libcamera  && \
    meson setup build --buildtype=release -Dpipelines=rpi/vc4,rpi/pisp -Dipas=rpi/vc4,rpi/pisp -Dv4l2=true -Dgstreamer=enabled -Dtest=false -Dlc-compliance=disabled -Dcam=disabled -Dqcam=disabled -Ddocumentation=disabled -Dpycamera=enabled  && \
    ninja -C build install  && \
    sudo ninja -C build install

# rpicam
RUN cd /tmp && git clone https://github.com/raspberrypi/rpicam-apps.git  && \
    cd /tmp/rpicam-apps/  && \
    sudo apt-get install -y cmake libboost-program-options-dev libdrm-dev libexif-dev && \
    sudo apt-get install -y ffmpeg libavcodec-extra libavcodec-dev libavdevice-dev libpng-dev libpng-tools libepoxy-dev && \
    sudo apt-get install -y qt5-qmake qtmultimedia5-dev && \
    meson setup build -Denable_libav=enabled -Denable_drm=enabled -Denable_egl=enabled -Denable_qt=enabled -Denable_opencv=disabled -Denable_tflite=disabled -Denable_hailo=disabled && \
    meson compile -C build && \
    sudo meson install -C build && \
    sudo ldconfig

# ros camera
RUN cd /root/colcon_ws/src \
  && git clone https://github.com/christianrauch/camera_ros.git \
  && cd /root/colcon_ws \
  && rosdep install -y --from-paths src/camera_ros --ignore-src --rosdistro humble --skip-keys=libcamera

# Clean up source artifacts
RUN rm -rf /home/orb/ORB_SLAM3 /root/colcon_ws
