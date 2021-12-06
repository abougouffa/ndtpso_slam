#include "ros/ros.h"
#include "ndtpso_slam_node.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ndtpso_slam");
  ros::NodeHandle nh("~");

  NDTPSONode node(nh);


}
