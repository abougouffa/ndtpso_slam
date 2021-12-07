#ifndef NDTPSO_SLAM_NODE_H_
#define NDTPSO_SLAM_NODE_H_

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "ndtpso_slam/ndtcell.h"
#include "ndtpso_slam/ndtframe.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <chrono>
#include <string>
#include <eigen3/Eigen/Core>
#include <mutex>
// #include <ctime>
// #include <iostream>

#define SAVE_MAP_DATA_TO_FILE true
#define SAVE_DATA_TO_FILE_EACH_NUM_ITERS 10
#define SAVE_MAP_IMAGES false

#define SYNC_WITH_LASER_TOPIC false
#define SYNC_WITH_ODOM false

#define DEFAULT_WAIT_FOR_TF true
#define DEFAULT_CELL_SIZE_M .5
#define DEFAULT_FRAME_SIZE_M 100
#define DEFAULT_SCAN_TOPIC std::string("/scan")
#define DEFAULT_SCAN_FRAME_ID std::string("scan")
#define DEFAULT_BASE_FRAME_ID std::string("base_link")
#define DEFAULT_ODOM_FRAME_ID std::string("odom")
#define DEFAULT_GLOBAL_FRAME_ID std::string("map")
#define DEFAULT_OUTPUT_MAP_SIZE_M 25
#define DEFAULT_RATE_HZ 10
#define DEFAULT_OCCUPANCY_GRID_CELL_SIZE_M .1

struct FrameTree {
  NDTFrame *frame{nullptr};
  Eigen::Vector3d pose{Eigen::Vector3d::Zero()};
  FrameTree *next{nullptr}, *prev{nullptr};
};

class NDTPSONode {
private:
  std::mutex matcher_mutex_;
  std::chrono::time_point<std::chrono::high_resolution_clock> start_time_,
      last_call_time_;

  NDTPSOConfig ndtpso_conf_; // Initally, the object helds the default values
  std::string param_base_frame_id_, param_global_frame_id_, param_scan_frame_id_, param_odom_frame_id_, param_scan_topic_;
  int param_frame_size_, param_map_size_m_, param_rate_;
  double param_cell_side_;

#if BUILD_OCCUPANCY_GRID
  double param_occupancy_grid_res_;
#endif

  bool first_iteration_{true};
  unsigned int number_of_iters_{0};
  Eigen::Vector3d previous_pose_{Eigen::Vector3d::Zero()},
      trans_estimate_{Eigen::Vector3d::Zero()},
      initial_pose_{Eigen::Vector3d::Zero()},
      current_pose_{Eigen::Vector3d::Zero()};

  NDTFrame *current_frame_;
  NDTFrame *ref_frame_;

  FrameTree *head, *last;

#if SAVE_MAP_DATA_TO_FILE
  NDTFrame *global_map_;
#endif

  ros::Publisher pose_pub_;
  ros::Subscriber laser_sub_;
  geometry_msgs::PoseStamped current_pose_msg_;

  void scan_matcher_(const sensor_msgs::LaserScanConstPtr &scan);

public:
  NDTPSONode(ros::NodeHandle& nh);
};

#endif // NDTPSO_SLAM_NODE_H_
