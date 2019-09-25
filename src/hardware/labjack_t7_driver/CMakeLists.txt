cmake_minimum_required(VERSION 2.8.3)
project(labjack_t7_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  robot_config
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  /usr/local/include
)

catkin_package()

add_library(labjack src/labjack.cpp)

add_executable(labjack_node src/labjack_node.cpp)

target_link_libraries(labjack_node 
                      labjack #Custom wrapper functions for LabJack
                      /usr/local/lib/libLabJackM.so #LabJack base lib
                      ${catkin_LIBRARIES})

install(TARGETS labjack_node labjack
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

