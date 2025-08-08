


## One Mono Camera
First run this to set up camera/image_raw topic:
```
source run_camera.sh
```

```
ros2 run camera_calibration cameracalibrator --size 6x8 --square 0.025 --no-service-check --ros-args -r image:=/camera/image_raw -r camera:=/camera
```
Set the size and squre params according to your printed chesstable

## If you have two camreas avaiable run these:

First run the two topics the same way as you would do with mono but for left and right:
```
ros2 run camera_ros camera_node --ros-args \
  -p camera_id:=0 \
  -p format:="R8" \
  -p role:="video" \
  -p width:=640 -p height:=400 \
  -r __ns:=/stereo/left \
  -r image_raw:=/left/image_raw \
  -p camera:="/base/axi/pcie@1000120000/rp1/i2c@88000/ov9281@60" \
  -r camera_info:=/left/camera_info
  -p fps:=20 \
  -p exposure_time:=8333 \
  -p frame_id:="camera"

# Terminal 2 - Right camera  
ros2 run camera_ros camera_node --ros-args \
  -p camera_id:=1 \
  -p pixel_format:=R8 \
  -p image_width:=1280 \
  -p image_height:=800 \
  -r __ns:=/stereo/right \
  -r image_raw:=/right/image_raw \
  -r camera_info:=/right/camera_info
```

Than run:

ros2 run camera_calibration cameracalibrator --size 6x8 --square 0.025 \
  --ros-args -r image:=/image_raw