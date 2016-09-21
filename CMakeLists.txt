cmake_minimum_required(VERSION 2.8.3)
project(odom_translation)
set(CMAKE_CXX_FLAGS "-std=c++11 -march=native -DEIGEN_DONT_PARALLELIZE")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
      cv_bridge
    camera_model
    image_transport
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
catkin_package()

add_executable(odom_translation_node src/odom_translation_node.cpp)
target_link_libraries(odom_translation_node ${catkin_LIBRARIES})