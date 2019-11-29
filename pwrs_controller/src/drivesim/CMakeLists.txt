cmake_minimum_required(VERSION 3.0.2)
project(pwrs_drivesim)

find_package(catkin REQUIRED
  roscpp
  tf
  ackermann_msgs
)

include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(drivesim
               main.cpp)

target_link_libraries(drivesim ${catkin_LIBRARIES})

install(TARGETS
  drivesim
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
