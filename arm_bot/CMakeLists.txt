cmake_minimum_required(VERSION 2.8.3)
project(arm_bot)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES arm_bot
  CATKIN_DEPENDS roscpp rospy tf
  DEPENDS system_lib
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${boost_INCLUDE_DIRS}
)

add_library(arm_bot
  src/arm_bot/arm_tele_op.cpp
)

add_executable(arm_tele_op src/arm_tele_op_node.cpp)

target_link_libraries(arm_tele_op
  arm_bot
  ${catkin_LIBRARIES}
)

install(DIRECTORY include/arm_bot/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS
  arm_bot
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

