remake_find_package(libmv CONFIG)
remake_find_package(libsnappy CONFIG)
remake_find_package(Threads)

remake_include(${LIBMV_INCLUDE_DIRS})
remake_include(${LIBSNAPPY_INCLUDE_DIRS})
remake_include()

remake_define(linux true)

remake_ros_package_add_library(mv-cameras RECURSE LINK ${CMAKE_THREAD_LIBS_INIT}
  ${LIBMV_LIBRARIES} ${LIBSNAPPY_LIBRARIES})
