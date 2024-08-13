cmake_minimum_required(VERSION 3.0.2)
project(monarch_ros)

find_package(catkin REQUIRED COMPONENTS
  monarch_sdk
  roscpp
)

find_package(PNG REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(camera_wrapper
  src/camera_wrapper.cpp
  src/utils.cpp
)

target_link_libraries(camera_wrapper
  ${catkin_LIBRARIES}
  png
)

include_directories(include)

install(TARGETS camera_wrapper
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
