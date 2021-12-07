#include "ndtpso_slam_node.hpp"
#include <cstdio>
#include <ctime>
#include <iostream>
#include <mutex>
#include <string>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>

// The odometry is used just for the initial pose to be easily compared with our
// calculated pose

NDTPSONode::NDTPSONode(ros::NodeHandle& nh) {
  // Read parameters
  nh.param<std::string>("scan_topic", param_scan_topic_, DEFAULT_SCAN_TOPIC);
  nh.param<std::string>("scan_frame_id", param_scan_frame_id_, DEFAULT_SCAN_FRAME_ID);
  nh.param<std::string>("base_frame_id", param_base_frame_id_, DEFAULT_BASE_FRAME_ID);
  nh.param<std::string>("odom_frame_id", param_odom_frame_id_, DEFAULT_ODOM_FRAME_ID);
  nh.param<std::string>("global_frame_id", param_global_frame_id_, DEFAULT_GLOBAL_FRAME_ID);
  nh.param("map_size_m", param_map_size_m_, DEFAULT_OUTPUT_MAP_SIZE_M);
  nh.param("rate_hz", param_rate_, DEFAULT_RATE_HZ);
  nh.param("cell_side", param_cell_side_, DEFAULT_CELL_SIZE_M);
  nh.param("frame_size", param_frame_size_, DEFAULT_FRAME_SIZE_M);

  double init_x, init_y, init_a;
  nh.param("init_pose_x", init_x, 0.);
  nh.param("init_pose_y", init_y, 0.);
  nh.param("init_pose_a", init_a, 0.);

  nh.param("pso_num_threads", ndtpso_conf_.psoConfig.num_threads, -1);
  nh.param("pso_iterations", ndtpso_conf_.psoConfig.iterations, PSO_ITERATIONS);
  nh.param("pso_population", ndtpso_conf_.psoConfig.populationSize, PSO_POPULATION_SIZE);
  nh.param("pso_c1", ndtpso_conf_.psoConfig.coeff.c1, PSO_C1);
  nh.param("pso_c2", ndtpso_conf_.psoConfig.coeff.c1, PSO_C2);
  nh.param("pso_w", ndtpso_conf_.psoConfig.coeff.w, PSO_W);
  nh.param("pso_w_dumping", ndtpso_conf_.psoConfig.coeff.w_dumping, PSO_W_DUMPING_COEF);

#if BUILD_OCCUPANCY_GRID
  nh.param("occupancy_grid_resolution_m", param_occupancy_grid_res_, DEFAULT_OCCUPANCY_GRID_CELL_SIZE_M);
#endif

  // Print patameters
  ROS_INFO("scan_topic:= \"%s\"", param_scan_topic_.c_str());
  ROS_INFO("scan_frame:= \"%s\"", param_scan_frame_id_.c_str());
  ROS_INFO("rate:= %dHz", param_rate_);
  ROS_INFO("Config [Max Map Size: %dx%dm]", param_map_size_m_, param_map_size_m_);
  ROS_INFO("Config [NDT Cell Size: %.2fm]", param_cell_side_);
  ROS_INFO("Config [NDT Frame Size: %dx%dm]", param_frame_size_, param_frame_size_);
  ROS_INFO("Config [NDT Window Size: %d]", NDT_WINDOW_SIZE);
  ROS_INFO("Config [PSO Number of Iterations: %d]", ndtpso_conf_.psoConfig.iterations);
  ROS_INFO("Config [PSO Population Size: %d]", ndtpso_conf_.psoConfig.populationSize);
  ROS_INFO("Config [PSO Threads: %d]", ndtpso_conf_.psoConfig.num_threads);
  ROS_INFO("Config [PSO C1: %.2f]", ndtpso_conf_.psoConfig.coeff.c1);
  ROS_INFO("Config [PSO C2: %.2f]", ndtpso_conf_.psoConfig.coeff.c2);
  ROS_INFO("Config [PSO W: %.2f]", ndtpso_conf_.psoConfig.coeff.w);
  ROS_INFO("Config [PSO W Dumping: %.2f]", ndtpso_conf_.psoConfig.coeff.w_dumping);
#if BUILD_OCCUPANCY_GRID
  ROS_INFO("Config [Occupancy Grid Cell Resolution: %.2fm]", param_occupancy_grid_res_);
#endif

  // The reference frame which will be used for all the matching operations,
  // It is the only frame which needs to be set to the correct cell and
  // occupancy grid sizes
  ref_frame_ = new NDTFrame(Vector3d::Zero(), static_cast<unsigned short>(param_frame_size_), static_cast<unsigned short>(param_frame_size_),
                            param_cell_side_, true, ndtpso_conf_
#if BUILD_OCCUPANCY_GRID
                            ,
                            param_occupancy_grid_res_
#endif
  );

#if SAVE_MAP_DATA_TO_FILE
  global_map_ = new NDTFrame(Vector3d::Zero(), static_cast<unsigned short>(param_map_size_m_), static_cast<unsigned short>(param_map_size_m_),
                             param_map_size_m_, false);
#endif

  current_frame_ = new NDTFrame(initial_pose_, static_cast<unsigned short>(param_frame_size_), static_cast<unsigned short>(param_frame_size_),
                                param_cell_side_, false);

  pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("pose", 1);

  // If the two frames are equal, there is no need to transform it
  if (param_base_frame_id_ != param_scan_frame_id_) {
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);
    geometry_msgs::TransformStamped tf_stamped;
    ROS_INFO("Waiting for tf \"%s\" -> \"%s\"", param_base_frame_id_.c_str(), param_scan_frame_id_.c_str());

    while (true) {
      try {
        tf_stamped = tf_buffer.lookupTransform(param_base_frame_id_, param_scan_frame_id_, ros::Time(0.0));
      } catch (tf2::TransformException& e) {
        ROS_WARN("Failed getting TF, error: %s", e.what());
        ros::Duration(2.0).sleep();
        continue;
      }

      break;
    }

    auto initial_trans =
        Vector3d(tf_stamped.transform.translation.x, tf_stamped.transform.translation.y, tf2::getYaw(tf_stamped.transform.rotation));

    ROS_INFO("Got and latched a tf (\"%s\" -> \"%s\") = (%.5f, %.5f, %.5f)", param_base_frame_id_.c_str(), param_scan_frame_id_.c_str(),
             initial_trans.x(), initial_trans.y(), initial_trans.z());

    // TODO: run scan matcher in the LiDAR frame,
    // and use the static tf 'transform' to convert the
    // estimated pose from 'scan' to 'base_link',
    current_frame_->setTrans(initial_trans);
  }

  // Initial pose
  initial_pose_ = previous_pose_ = current_pose_ = Eigen::Vector3d(init_x, init_y, init_a);

  ROS_INFO("Starting from initial pose (%.5f, %.5f, %.5f)", initial_pose_.x(), initial_pose_.y(), initial_pose_.z());

  laser_sub_ = nh.subscribe<sensor_msgs::LaserScan>(param_scan_topic_, 1, &NDTPSONode::scan_matcher_, this);

  ROS_INFO("NDTPSO node started successfuly");

  // Using the ros::Rate + ros::spinOnce can slows down the ApproxSyncPolicy if
  // no sufficient buffer
  ros::Rate loop_rate(param_rate_);

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  char time_formated[80];
  time_t rawtime;
  time(&rawtime);

  tm* timeinfo = localtime(&rawtime);

  strftime(time_formated, sizeof(time_formated), "%Y%m%d-%H%M%S", timeinfo);

  std::cout << std::endl
            << "Exporting results "
            << "[.pose.csv, .map.csv, .png, .gnuplot]" << std::endl;

  size_t pos = 0;

  do {
    param_scan_topic_.replace(pos, 1, "_");
    pos = param_scan_topic_.find("/", pos);
  } while (pos != std::string::npos);

  char filename[512];

  sprintf(filename, "%s-%s", param_scan_topic_.c_str(), time_formated);

#if SAVE_MAP_DATA_TO_FILE
  global_map_->dumpMap(filename, true, true, SAVE_MAP_IMAGES, 100
#if BUILD_OCCUPANCY_GRID
                       ,
                       true
#endif
  );

  std::cout << "Map saved to file " << filename << "[.pose.csv, .map.csv, .png, .gnuplot]" << std::endl;
#endif

  sprintf(filename, "%s-%s-ref-frame", param_scan_topic_.c_str(), time_formated);

  ref_frame_->dumpMap(filename, false, true, SAVE_MAP_IMAGES, 100
#if BUILD_OCCUPANCY_GRID
                      ,
                      true
#endif
  );

  std::cout << "Map saved to file " << filename << "[.pose.csv, .map.csv, .png, .gnuplot]" << std::endl;
}

void NDTPSONode::scan_matcher_(const sensor_msgs::LaserScanConstPtr& scan) {
#if SAVE_MAP_DATA_TO_FILE
  static unsigned int iter_num = 0;
#endif

  matcher_mutex_.lock();
  auto start = std::chrono::high_resolution_clock::now();
  last_call_time_ = start;

  current_frame_->loadLaser(scan->ranges, scan->angle_min, scan->angle_increment, scan->range_max);

  if (first_iteration_) {
    current_pose_ = previous_pose_;
    start_time_ = std::chrono::high_resolution_clock::now();
    ROS_INFO("Min/Max ranges: (%.2f, %.2f)", static_cast<double>(scan->range_min), static_cast<double>(scan->range_max));
    ROS_INFO("Min/Max angles: (%.2f, %.2f)", static_cast<double>(scan->angle_min), static_cast<double>(scan->angle_max));
  } else {
    current_pose_ = ref_frame_->align(previous_pose_, current_frame_);
  }

  previous_pose_ = current_pose_;
  ref_frame_->update(current_pose_, current_frame_);

#if SAVE_MAP_DATA_TO_FILE
  if (iter_num == 0) {
    global_map_->update(current_pose_, current_frame_);
  }

  iter_num = (iter_num + 1) % SAVE_DATA_TO_FILE_EACH_NUM_ITERS;
  global_map_->addPose(scan->header.stamp.toSec(), current_pose_);
#endif

  // Publish 'pose', using the same timestamp of the laserscan (or use
  // ros::Time::now() !!)
  current_pose_msg_.header.stamp = scan->header.stamp;
  current_pose_msg_.header.frame_id = param_global_frame_id_; // we can read it from config
  current_pose_msg_.pose.position.x = current_pose_.x();
  current_pose_msg_.pose.position.y = current_pose_.y();
  current_pose_msg_.pose.position.z = 0.;

  tf2::Quaternion q_ori;
  q_ori.setRPY(0, 0, current_pose_.z());
  current_pose_msg_.pose.orientation.x = q_ori.getX();
  current_pose_msg_.pose.orientation.y = q_ori.getY();
  current_pose_msg_.pose.orientation.z = q_ori.getZ();
  current_pose_msg_.pose.orientation.w = q_ori.getW();

  pose_pub_.publish(current_pose_msg_);

  // Reallocate the current_frame object, this is much faster than calling
  // current_frame->resetCells()
  delete current_frame_;
  current_frame_ = new NDTFrame(initial_pose_, static_cast<unsigned short>(param_frame_size_), static_cast<unsigned short>(param_frame_size_),
                                param_frame_size_, false);

  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;

  ++number_of_iters_;
  std::chrono::duration<double> current_rate = last_call_time_ - start_time_;

  if (!first_iteration_) {
    ROS_INFO("Average publish rate: %.2fHz, matching rate: %.2fHz", 1. / (current_rate.count() / number_of_iters_), 1. / elapsed.count());
  }

  first_iteration_ = false;
  matcher_mutex_.unlock();
}
