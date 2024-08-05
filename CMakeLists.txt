cmake_minimum_required(VERSION 3.0.2)
project(ros_monarch-pro)

find_package(catkin REQUIRED COMPONENTS
#   std_srvs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
)

# include_directories(
#   ${catkin_INCLUDE_DIRS}
# )

install(
  DIRECTORY 
    config
    launch
    lib
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(
  PROGRAMS 
    scripts/camera_wrapper
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)