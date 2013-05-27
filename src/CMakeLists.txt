remake_ros_package(
  mv_cameras
  DEPENDS roscpp diagnostic_updater std_msgs sensor_msgs image_transport
  DESCRIPTION "Driver for Matrix Vision cameras."
  EXTRA_BUILD_DEPENDS libmv-dev libsnappy-dev
  EXTRA_RUN_DEPENDS libmv libsnappy
)
