cmake_minimum_required(VERSION 3.8)
project(touch_control)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(touch_msgs REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(HD REQUIRED)

add_executable(touch_control src/touch_control_node.cpp src/hd++.cpp src/hd++.h)
target_compile_options(touch_control PRIVATE -Wall -Wextra)
ament_target_dependencies(touch_control rclcpp std_msgs geometry_msgs sensor_msgs touch_msgs Boost Eigen3 HD)
target_include_directories(touch_control PRIVATE ${EIGEN3_INCLUDE_DIRS})
# target_link_libraries(touch_control Boost::headers Eigen3::Eigen OpenHaptics::HD
    # rclcpp std_msgs geometry_msgs::geometry_msgs sensor_msgs touch_msgs)

install(
  TARGETS touch_control
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
