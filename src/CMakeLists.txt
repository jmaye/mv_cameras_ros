remake_ros_package(
  mv_cameras
  DEPENDS roscpp diagnostic_updater
  DESCRIPTION "Driver for Matrix Vision cameras."
  EXTRA_BUILD_DEPENDS libmv-dev libsnappy-dev
  EXTRA_RUN_DEPENDS libmv libsnappy
)
