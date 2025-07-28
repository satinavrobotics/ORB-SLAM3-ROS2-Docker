ros2 run camera_ros camera_node --ros-args \
  -p format:="R8" \
  -p role:="video" \
  -p width:=640 -p height:=400 \
  -p fps:=20 \
  -p exposure_time:=8333 \
  -p camera:="/base/axi/pcie@1000120000/rp1/i2c@88000/ov9281@60" \
  -p frame_id:="camera"
