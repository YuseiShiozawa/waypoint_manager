cmake_minimum_required(VERSION 3.0.2)
project(check_robot_moving VERSION 0.0.0 LANGUAGES CXX)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf
    geometry_msgs
    waypoint_manager_msgs
    costmap_2d
)
link_directories(${catkin_LIBRARY_DIRS})
## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Executable for check_robot_moving_node
add_executable(check_robot_moving_node
    ${CMAKE_CURRENT_SOURCE_DIR}/src/check_robot_moving_node.cpp
)
set_target_properties(check_robot_moving_node
    PROPERTIES
        OUTPUT_NAME check_robot_moving_node
        VERSION ${PROJECT_VERSION}
)

# Executable for stop_mode_node
add_executable(scan_check_node src/scan_check.cpp)

# C++14 settings for check_robot_moving_node
target_compile_features(check_robot_moving_node
    PRIVATE
        cxx_std_14
)

## Add necessary links for check_robot_moving_node
target_link_libraries(check_robot_moving_node
    Eigen3::Eigen
    ${catkin_LIBRARIES}
)

## Add necessary links for scan_check_node
target_link_libraries(scan_check_node
    ${catkin_LIBRARIES}
    ${catkin_INCLUDE_DIRS}
    costmap_2d
)

# In case you need to enforce C++14
# add_compile_options(-std=c++14)
