cmake_minimum_required(VERSION 3.0.2)
project(pwrs_description)

find_package(catkin REQUIRED)

catkin_package()

file(DOWNLOAD
   https://raw.githubusercontent.com/gabomdq/SDL_GameControllerDB/master/gamecontrollerdb.txt
   ${PROJECT_SOURCE_DIR}/launch/gamecontrollerdb.txt
   EXPECTED_HASH SHA512=226df49456270605ac388ee7cfe6edd60a51d60150031ca35ccc0ac1d4b88602afe6a4de1b10906cdeb39988653faa1e5275d6c517110ee139490fb3d501c43b
   SHOW_PROGRESS
)    


install(DIRECTORY launch/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
   FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY urdf/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
   FILES_MATCHING PATTERN "*.xacro" PATTERN "*.yaml" PATTERN "*.urdf" PATTERN "*.stl"
)