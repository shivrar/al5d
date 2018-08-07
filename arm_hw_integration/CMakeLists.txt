cmake_minimum_required(VERSION 2.8.3)
project(arm_hw_integration)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  serial
  control_msgs
  actionlib
  actionlib_msgs
  sensor_msgs
)

generate_messages(
  DEPENDENCIES actionlib_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy serial actionlib actionlib_msgs
)


include_directories(
 include
 ${boost_INCLUDE_DIRS}
 ${catkin_INCLUDE_DIRS}
)

add_executable(serial_test src/test.cpp)

add_library(arm_hw_integration
   src/arm_hw_integration/hw_action_server.cpp
)

target_link_libraries(arm_hw_integration
   ${catkin_LIBRARIES}
   ${boost_INCLUDE_DIRS}

)

target_link_libraries(serial_test
   arm_hw_integration
)
