cmake_minimum_required(VERSION 3.0.2)
project(pwrs_wiggler)

find_package(catkin REQUIRED
  roscpp
  ackermann_msgs
)

include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(wiggler
               main.cpp)

target_link_libraries(wiggler ${catkin_LIBRARIES})

install(TARGETS
  wiggler
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
