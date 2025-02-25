# Minimum CMake version
cmake_minimum_required(VERSION 3.16)

# Project name and language
project(nmpcpidquad LANGUAGES C CXX)

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

# Include directories
include_directories(
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/qpoases
    ${CMAKE_SOURCE_DIR}/qpoases/INCLUDE
    ${CMAKE_SOURCE_DIR}/qpoases/SRC
    ${CMAKE_SOURCE_DIR}/include
)

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
)
include_directories(/usr/local/include/acado)

link_directories(${GAZEBO_LIBRARY_DIRS})

# Source files
set(SOURCES
    ./qpoases/SRC/Bounds.cpp
    ./qpoases/SRC/Constraints.cpp
    ./qpoases/SRC/CyclingManager.cpp
    ./qpoases/SRC/Indexlist.cpp
    ./qpoases/SRC/MessageHandling.cpp
    ./qpoases/SRC/QProblem.cpp
    ./qpoases/SRC/QProblemB.cpp
    ./qpoases/SRC/SubjectTo.cpp
    ./qpoases/SRC/Utils.cpp
    ./qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
    ./include/acado_qpoases_interface.cpp
    ./include/acado_integrator.c
    ./include/acado_solver.c
    ./include/acado_auxiliary_functions.c
)

# Add the ROS2 node
add_executable(nmpcpidquad ${SOURCES} src/nmpcpidquad.cpp)
target_link_libraries(nmpcpidquad m stdc++)
ament_target_dependencies(nmpcpidquad rclcpp std_msgs tf2_ros nav_msgs)

add_executable(cascadedpid src/cascadedpid.cpp)
ament_target_dependencies(cascadedpid rclcpp std_msgs tf2_ros nav_msgs)

add_library(uavplugin SHARED src/uavplugin.cc)
ament_target_dependencies(uavplugin rclcpp std_msgs)
target_link_libraries(uavplugin ${GAZEBO_LIBRARIES} rclcpp::rclcpp)

# Install the node
install(TARGETS
    nmpcpidquad
    cascadedpid
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
