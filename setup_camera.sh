sudo apt-get install -y clang meson ninja-build pkg-config libyaml-dev python3-yaml python3-ply python3-jinja2 openssl
sudo apt-get install -y libdw-dev libunwind-dev libudev-dev libudev-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libpython3-dev pybind11-dev libevent-dev libtiff-dev qt6-base-dev qt6-tools-dev-tools liblttng-ust-dev python3-jinja2 lttng-tools libexif-dev libjpeg-dev pybind11-dev libevent-dev libgtest-dev abi-compliance-checker

# libcamera
git clone https://github.com/raspberrypi/libcamera.git
cd libcamera
meson setup build --buildtype=release -Dpipelines=rpi/vc4,rpi/pisp -Dipas=rpi/vc4,rpi/pisp -Dv4l2=true -Dgstreamer=enabled -Dtest=false -Dlc-compliance=disabled -Dcam=disabled -Dqcam=disabled -Ddocumentation=disabled -Dpycamera=enabled
ninja -C build install
sudo ninja -C build install

# rpicam
git clone https://github.com/raspberrypi/rpicam-apps.git
cd rpicam-apps/
sudo apt install cmake libboost-program-options-dev libdrm-dev libexif-dev
sudo apt install ffmpeg libavcodec-extra libavcodec-dev libavdevice-dev libpng-dev libpng-tools libepoxy-dev 
sudo apt install qt5-qmake qtmultimedia5-dev
meson setup build -Denable_libav=enabled -Denable_drm=enabled -Denable_egl=enabled -Denable_qt=enabled -Denable_opencv=disabled -Denable_tflite=disabled -Denable_hailo=disabled
meson compile -C build
sudo meson install -C build
sudo ldconfig
rpicam-still --version