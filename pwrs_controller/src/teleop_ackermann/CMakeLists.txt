cmake_minimum_required(VERSION 3.0.2)
project(teleop_ackermann)

find_package(catkin REQUIRED
  roscpp
  ackermann_msgs
  sensor_msgs
)

include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(teleop_ackermann_node
               main.cpp)

target_link_libraries(teleop_ackermann_node ${catkin_LIBRARIES})

install(TARGETS
  teleop_ackermann_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
