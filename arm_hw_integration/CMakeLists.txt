cmake_minimum_required(VERSION 2.8.3)
project(arm_hw_integration)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  serial
)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy serial
)


include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_executable(serial_test src/test.cpp)

target_link_libraries(serial_test
   ${catkin_LIBRARIES}
)
