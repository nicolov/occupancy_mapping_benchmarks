
cmake_minimum_required(VERSION 2.8.3)
project(octomap_catkin)

find_package(catkin REQUIRED)

include(ExternalProject)
file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)
ExternalProject_Add(octomap_src
  URL "${CMAKE_CURRENT_LIST_DIR}/../octomap"
  CMAKE_ARGS
    -DCMAKE_INSTALL_PREFIX:PATH=${CATKIN_DEVEL_PREFIX}
)

add_library(octomap_catkin lib_wrap.cc)
add_dependencies(octomap_catkin octomap_src)

catkin_package(
    LIBRARIES ${PROJECT_NAME}_catkin
)
