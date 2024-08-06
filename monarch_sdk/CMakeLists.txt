cmake_minimum_required(VERSION 3.0.2)
project(monarch_sdk)

find_package(catkin REQUIRED)

find_package(OpenCV REQUIRED COMPONENTS
  core
  highgui
)

find_package(PNG REQUIRED)

if(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64")
  set(UCAM_LIBRARY_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib/x86_64)
elseif(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64")
  set(UCAM_LIBRARY_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib/aarch64)
else()
  message(FATAL_ERROR "Unsupported platform: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

find_library(UCAM NAMES "UCam" PATHS ${UCAM_LIBRARY_PATH})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES UCam
)

add_library(UCam EXCLUDE_FROM_ALL ${UCAM})

add_custom_command(
  TARGET UCam
  POST_BUILD
  COMMAND cp ${UCAM} ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/
)

set_target_properties(UCam PROPERTIES LINKER_LANGUAGE CXX)

add_executable(TestSdk
  src/TestSdk.cpp
)

target_link_libraries(TestSdk
  UCam
  opencv_core
  opencv_highgui
  png
)

include_directories(include)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  FILES ${UCAM}
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS TestSdk
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
