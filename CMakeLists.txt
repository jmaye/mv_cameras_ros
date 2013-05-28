cmake_minimum_required(VERSION 2.6)

include(ReMake)

remake_project(
  mv-cameras-ros
  VERSION 0.1
  RELEASE alpha
  SUMMARY "ROS driver for Matrix Vision cameras."
  AUTHOR "Jerome Maye"
  CONTACT "jerome.maye@mavt.ethz.ch"
  HOME "http://github.com/jmaye/mv_cameras_ros"
  LICENSE "GNU Lesser General Public License (LGPL)"
  PREFIX mv-cameras-
  INSTALL /usr
  CONFIGURATION_DESTINATION /etc/mv-cameras
  EXTRA_CXX_FLAGS -std=c++0x
)

remake_ros_pack_deb()

remake_ros_distribute_deb(
  DISTRIBUTION lucid
  SECTION libs
  UPLOAD ppa:jmaye/ros
  PASS CMAKE_BUILD_TYPE MV_CAMERAS_ROS_GIT_REVISION
)

remake_ros_distribute_deb(
  DISTRIBUTION precise
  SECTION libs
  UPLOAD ppa:jmaye/ros
  PASS CMAKE_BUILD_TYPE MV_CAMERAS_ROS_GIT_REVISION
)
