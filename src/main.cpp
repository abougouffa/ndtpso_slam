#include "ndtpso_slam_node.hpp"
#include <cstdlib>
#include <stdexcept>
#include <ros/ros.h>
#include <thread>

using namespace std::chrono_literals;

int main(int argc, char** argv) {
  ros::init(argc, argv, "ndtpso_slam");

  while (!ros::master::check()) {
    ROS_ERROR("Waiting for ROS master at %s", ros::master::getURI().c_str());
    std::this_thread::sleep_for(1s);
  }

  ROS_INFO("Connected to ROS master");

  ros::NodeHandle nh("~");

  try {
    NDTPSONode node(nh);
  } catch (std::runtime_error& ex) {
    ROS_ERROR("FATAL ERROR, exception occured '%s'", ex.what());
    std::exit(EXIT_FAILURE);
  }

  std::exit(EXIT_SUCCESS);
}
