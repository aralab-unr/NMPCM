# Minimum CMake version
cmake_minimum_required(VERSION 3.16)

# Project name and language
project(nmpccasadiquad LANGUAGES C CXX)

# Find ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_msgs REQUIRED)


# Compiler flags
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

find_library(CASADI_LIB casadi REQUIRED PATHS /usr/local/lib /usr/lib)
include_directories(/usr/local/include)

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

# Add the ROS2 node
add_executable(nmpccasadiquad src/nmpccasadiquad.cpp)
target_link_libraries(nmpccasadiquad ${CASADI_LIB})
ament_target_dependencies(nmpccasadiquad rclcpp std_msgs tf2_ros nav_msgs)

add_library(uavplugin SHARED src/uavplugin.cc)
ament_target_dependencies(uavplugin rclcpp std_msgs)
target_link_libraries(uavplugin ${GAZEBO_LIBRARIES} rclcpp::rclcpp)

# Install the node
install(TARGETS
    nmpccasadiquad
    uavplugin
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# Export package
ament_package()
