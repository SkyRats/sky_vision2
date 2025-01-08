cmake_minimum_required(VERSION 3.8)
project(sky_vision2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic )
endif()


# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(cv_bridge REQUIRED)
find_package(OpenCV 4.11 REQUIRED)
find_package(image_transport REQUIRED)


# Include Cpp "include" directory
include_directories(include)


# Create Cpp executable
add_executable(aruco_node src/aruco_detector_node.cpp)
ament_target_dependencies(aruco_node rclcpp geometry_msgs OpenCV cv_bridge sensor_msgs std_msgs)

# Install Cpp executables
install(TARGETS
  aruco_node
  DESTINATION lib/${PROJECT_NAME}
)


# Install Python modules
ament_python_install_package(${PROJECT_NAME})


# Install Python executables
install(PROGRAMS
  scripts/main.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
