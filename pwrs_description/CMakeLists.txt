cmake_minimum_required(VERSION 3.0.2)
project(pwrs_description)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
   FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY urdf/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
   FILES_MATCHING PATTERN "*.xacro" PATTERN "*.yaml" PATTERN "*.urdf" PATTERN "*.stl"
)