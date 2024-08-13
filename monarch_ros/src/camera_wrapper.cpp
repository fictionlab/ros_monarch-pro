#include <signal.h>

#include "leo_msgs/MonarchRosSavePath.h"
#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "utils.hpp"

static ros::ServiceServer save_images_service;
// static std::unique_ptr<CameraCardController> myCamera;
static unsigned short bufRunLUTFrames[10 * 1024 * 1280];
static int bufRunLUTFramesSize = 10;
static CameraWrapper *wrapper;

bool save_images_callback(leo_msgs::MonarchRosSavePathRequest &req,
                          leo_msgs::MonarchRosSavePathResponse &res) {
  std::string path = "";
  std::stringstream message;

  message << "Files saved in ";

  if (req.path == "") {
    ROS_INFO("Empty path provided - saving content in home directory");
    path = getenv("HOME");
  } else {
    path = req.path;
  }

  message << path;

  if (wrapper->CaptureLUTFrames(path)) {
    res.success = true;
    res.message = message.str();
    return true;
  } else {
    res.success = false;
    res.message = "Failed to save the images.";
    return false;
  }
}

void log_camera_info() {
  std::stringstream data(wrapper->DisplayAllData());
  std::string line;
  while (std::getline(data, line, '\n')) {
    ROS_INFO_STREAM(line);
  }
}

void sigint_handler(int sig) {
  delete wrapper;
  ros::shutdown();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "camera_wrapper");

  ros::NodeHandle pnh("~");
  int fps = pnh.param("exposure_fps", 100);
  float gain = pnh.param("gain", 1.0);

  wrapper = new CameraWrapper(fps, gain);

  if (wrapper->cameraFound) {
    ROS_INFO("CAMERA FOUND");
    log_camera_info();

    save_images_service =
        pnh.advertiseService("save_images", &save_images_callback);

    signal(SIGINT, sigint_handler);

    ros::spin();
  } else {
    ROS_INFO("CAMERA NOT FOUND");
    delete wrapper;
  }

  return 0;
}