#include "ros/ros.h"

#include "monarch_sdk/CameraCardController.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_node");

  ros::NodeHandle nh;

  CameraCardController myCamera;

  if (myCamera.IsCameraFound()) {
    ROS_INFO("CAMERA FOUND");
  } else {
    ROS_INFO("CAMERA NOT FOUND");
  }

  ros::spin();
  return 0;
}