cmake_minimum_required(VERSION 2.8.3)
project(occupancy_mapping_benchmarks)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/")

find_package(catkin REQUIRED COMPONENTS
    rosbag
    rosfmt
    backward_ros
    sensor_msgs
    tf
    pcl_ros
    pcl_conversions
    octomap_ros
    tf2_ros
    skimap_ros
)

find_package(octomap REQUIRED)
find_package(OpenVDB REQUIRED)
find_package(TBB REQUIRED)

catkin_package()

add_compile_options (-fdiagnostics-color=always)

add_executable(benchmark
        src/benchmark.cc
)
target_include_directories(benchmark PUBLIC
        ${catkin_INCLUDE_DIRS}
        ${OCTOMAP_INCLUDE_DIRS}
        ${OpenVDB_INCLUDE_DIR}
)
target_link_libraries(benchmark
        ${catkin_LIBRARIES}
        ${OCTOMAP_LIBRARIES}
        ${OpenVDB_LIBRARIES}
        ${TBB_LIBRARIES}
        # Sad times..
        -lHalf
)
