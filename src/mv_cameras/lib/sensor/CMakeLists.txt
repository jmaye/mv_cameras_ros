remake_find_package(libmv CONFIG)
remake_find_package(libsnappy CONFIG)

remake_include(${LIBMV_INCLUDE_DIRS})
remake_include(${LIBSNAPPY_INCLUDE_DIRS})

remake_define(linux true)

remake_ros_package_add_library(cameras LINK base ${LIBMV_LIBRARIES}
  ${LIBSNAPPY_LIBRARIES})
