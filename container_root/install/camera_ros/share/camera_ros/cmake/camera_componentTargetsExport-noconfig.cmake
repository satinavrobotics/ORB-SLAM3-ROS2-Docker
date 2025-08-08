#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "camera_ros::camera_component" for configuration ""
set_property(TARGET camera_ros::camera_component APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(camera_ros::camera_component PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libcamera_component.so"
  IMPORTED_SONAME_NOCONFIG "libcamera_component.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS camera_ros::camera_component )
list(APPEND _IMPORT_CHECK_FILES_FOR_camera_ros::camera_component "${_IMPORT_PREFIX}/lib/libcamera_component.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
