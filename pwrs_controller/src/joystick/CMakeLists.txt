cmake_minimum_required(VERSION 3.0.2)
project(pwrs_joystick)

find_package(catkin REQUIRED
  roscpp
  ackermann_msgs
)

include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(joystick
               main.cpp)

target_link_libraries(joystick ${catkin_LIBRARIES} winmm.lib)

install(TARGETS
  joystick
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
