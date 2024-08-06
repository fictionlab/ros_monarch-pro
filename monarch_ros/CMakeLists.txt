cmake_minimum_required(VERSION 3.0.2)
project(monarch_ros)

find_package(catkin REQUIRED COMPONENTS
  monarch_sdk
  roscpp
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(test_node
  src/test_node.cpp
)

target_link_libraries(test_node
  ${catkin_LIBRARIES}
)

install(TARGETS test_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
