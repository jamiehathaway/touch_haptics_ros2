cmake_minimum_required(VERSION 3.1)
project(touch_control)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs geometry_msgs touch_msgs)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(HD REQUIRED)

catkin_package()

add_executable(touch_control src/touch_control_node.cpp src/hd++.cpp)
target_compile_options(touch_control PRIVATE -Wall -Wextra)
target_include_directories(touch_control PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(touch_control Boost::headers Eigen3::Eigen OpenHaptics::HD ${catkin_LIBRARIES})
