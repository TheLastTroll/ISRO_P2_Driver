cmake_minimum_required(VERSION 3.8)
project(ISRO_P2_Driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# C Driver Library
add_library(ISRO_P2_Driver_lib SHARED
  src/ISRO_P2_Driver.c
)
target_include_directories(ISRO_P2_Driver_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(ISRO_P2_Driver_lib pthread)

# ROS2 Node Executable
add_executable(ISRO_P2_Driver_node 
  src/ISRO_P2_Driver_node.cpp
)
target_include_directories(ISRO_P2_Driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(ISRO_P2_Driver_node 
  ISRO_P2_Driver_lib
)
ament_target_dependencies(ISRO_P2_Driver_node
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

# Install Targets
install(TARGETS
  ISRO_P2_Driver_lib
  ISRO_P2_Driver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/ntrip.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install Directories
install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()