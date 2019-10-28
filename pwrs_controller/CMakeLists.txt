cmake_minimum_required(VERSION 3.0.2)
project(pwrs_controller)

find_package(catkin REQUIRED COMPONENTS roscpp tf)

catkin_package(CATKIN_DEPENDS roscpp std_msgs)

include_directories(${catkin_INCLUDE_DIRS})

add_subdirectory(src/joystick)
add_subdirectory(src/wiggler)
add_subdirectory(src/drivesim)