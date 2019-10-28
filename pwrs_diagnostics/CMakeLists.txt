cmake_minimum_required(VERSION 3.0.2)
project(pwrs_diagnostics)

find_package(Boost REQUIRED COMPONENTS system)

find_package(catkin REQUIRED COMPONENTS roscpp rosbag)

catkin_package(CATKIN_DEPENDS roscpp std_msgs rosbag)

include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

# Build the AppInsights library
set(AI_CORE app_insights/src/core)
include_directories(${AI_CORE})
add_library(app_insights SHARED ${AI_CORE}/BaseTelemetryContext.cpp ${AI_CORE}/TelemetryClient.cpp
			${AI_CORE}/TelemetryContextOther.cpp ${AI_CORE}/TelemetryContextWin32.cpp
			${AI_CORE}/channel/TelemetryChannel.cpp 
			${AI_CORE}/channel/utils/HttpHeaderField.cpp ${AI_CORE}/channel/utils/HttpRequest.cpp
			${AI_CORE}/channel/utils/HttpResponse.cpp
			${AI_CORE}/common/JsonWriter.cpp ${AI_CORE}/common/Serializer.cpp
			${AI_CORE}/common/StreamWriter.cpp ${AI_CORE}/common/StringWriter.cpp
			${AI_CORE}/common/Utils.cpp
			${AI_CORE}/contracts/Application.cpp ${AI_CORE}/contracts/Base.cpp ${AI_CORE}/contracts/CrashData.cpp
			${AI_CORE}/contracts/CrashDataBinary.cpp ${AI_CORE}/contracts/CrashDataHeaders.cpp
			${AI_CORE}/contracts/CrashDataThread.cpp ${AI_CORE}/contracts/CrashDataThreadFrame.cpp
			${AI_CORE}/contracts/Data.cpp ${AI_CORE}/contracts/DataPoint.cpp ${AI_CORE}/contracts/Device.cpp
			${AI_CORE}/contracts/Domain.cpp ${AI_CORE}/contracts/Envelope.cpp ${AI_CORE}/contracts/EventData.cpp
			${AI_CORE}/contracts/ExceptionData.cpp ${AI_CORE}/contracts/ExceptionDetails.cpp
			${AI_CORE}/contracts/Internal.cpp ${AI_CORE}/contracts/Location.cpp ${AI_CORE}/contracts/MessageData.cpp
			${AI_CORE}/contracts/MetricData.cpp ${AI_CORE}/contracts/Operation.cpp ${AI_CORE}/contracts/PageViewData.cpp
			${AI_CORE}/contracts/PageViewPerfData.cpp ${AI_CORE}/contracts/RemoteDependencydata.cpp
			${AI_CORE}/contracts/RequestData.cpp ${AI_CORE}/contracts/Session.cpp ${AI_CORE}/contracts/SessionStateData.cpp
			${AI_CORE}/contracts/StackFrame.cpp ${AI_CORE}/contracts/User.cpp)

set_target_properties(app_insights PROPERTIES COMPILE_DEFINITIONS 
		"TELEMETRY_CLIENT_DLL_EXPORT;UNICODE;_UNICODE;CPP_LIB_DEBUG;CURL_STATICLIB;WIN32;NDEBUG;_LIB;")

target_link_libraries(app_insights WinHttp)

install(TARGETS
  app_insights
  ARCHIVE DESTINATION ${CATKIN_GLOBAL_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_GLOBAL_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Build the Diagnostics Storage Tool/AI Uploader
include_directories(include)
add_executable(ai_uploader src/serializer.cpp src/serializer_node.cpp)
target_link_libraries(ai_uploader app_insights ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS
  ai_uploader
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
