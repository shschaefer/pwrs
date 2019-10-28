add_executable(joystick
               main.cpp)
target_link_libraries(joystick ${catkin_LIBRARIES} winmm.lib)

install(TARGETS
  joystick
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
