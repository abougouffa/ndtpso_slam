#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "ndtpso_slam/ndtcell.h"
#include "ndtpso_slam/ndtframe.h"
#include "ros/ros.h"
#include <chrono>
#include <cstdio>
#include <eigen3/Eigen/Core>
#include <iostream>
#include <laser_geometry/laser_geometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <mutex>
// #include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#define SAVE_DATA_TO_FILE true
#define CELL_SIZE .4
#define SIDE_M 200

using namespace Eigen;
using std::cout;
using std::endl;

static std::mutex matcher_mutex;
static bool first_iter;
static Vector3d global_trans, previous_pose, trans_estimate, initial_trans;

static NDTFrame* current_frame;
static NDTFrame ref_frame(Vector3d::Zero(), 50, 50, CELL_SIZE);

#if defined(SAVE_DATA_TO_FILE) && SAVE_DATA_TO_FILE
static NDTFrame* global_map;
//static FILE* csv_poses_file;
#endif

static geometry_msgs::PoseStamped current_pub_pose, pose2;
static ros::Publisher pose_pub;
static ros::Subscriber laser_sub;
static tf::TransformListener* tf_listener;

void scan_mathcher(const sensor_msgs::LaserScanConstPtr& scan, const nav_msgs::OdometryConstPtr& odom)
{
#if defined(SAVE_DATA_TO_FILE) && SAVE_DATA_TO_FILE
    static unsigned int iter_num = 0;
#endif
    matcher_mutex.lock();
    ros::Rate scan_loop_rate(10);
    auto start = std::chrono::high_resolution_clock::now();

    current_frame->loadLaser(scan->ranges, scan->angle_min, scan->angle_increment, scan->range_max);

    Vector3d current_pose;

    double _rx, _ry, _rz;
    tf::Matrix3x3(tf::Quaternion(
                      odom->pose.pose.orientation.x,
                      odom->pose.pose.orientation.y,
                      odom->pose.pose.orientation.z,
                      odom->pose.pose.orientation.w))
        .getRPY(_rx, _ry, _rz);

    if (first_iter) {
        first_iter = false;
        current_pose << odom->pose.pose.position.x, odom->pose.pose.position.y, _rz;
        previous_pose << odom->pose.pose.position.x, odom->pose.pose.position.y, _rz;
    } else {
        current_pose = ref_frame.align(previous_pose, current_frame);
    }

    previous_pose = current_pose;

    // ref_frame.resetPoints();
    ref_frame.update(current_pose, current_frame);

    auto finish = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> elapsed = finish - start;

    // std::cout << endl << "Elapsed time: " << elapsed.count() << " s\n";
    printf("t: %.5f\n", elapsed.count());

#if defined(SAVE_DATA_TO_FILE) && SAVE_DATA_TO_FILE
    if (iter_num == 0)
        global_map->update(current_pose, current_frame);
    iter_num = (iter_num + 1) % 10;
#endif

    // auto current_trans = current_pose - previous_pose;

    current_pub_pose.header.stamp = scan->header.stamp; // ros::Time::now();
    current_pub_pose.header.frame_id = "base_link"; // From config,
    current_pub_pose.pose.position.x = current_pose[0];
    current_pub_pose.pose.position.y = current_pose[1];
    current_pub_pose.pose.position.z = 0;
    tf::Quaternion q_ori;
    q_ori.setRPY(0, 0, current_pose[2]);
    current_pub_pose.pose.orientation.x = q_ori.getX();
    current_pub_pose.pose.orientation.y = q_ori.getY();
    current_pub_pose.pose.orientation.z = q_ori.getZ();
    current_pub_pose.pose.orientation.w = q_ori.getW();
    // tf_listener->transformPose("base_link", current_pub_pose, pose2);
    pose_pub.publish(current_pub_pose);

    // static tf::TransformBroadcaster br;
    // geometry_msgs::TransformStamped transformStamped;

    // tf::Transform transform;
    // transform.setOrigin(tf::Vector3(current_pose[0], current_pose[1], 0.0));
    // tf::Quaternion q;
    // q.setRPY(0, 0, current_pose[2]);
    // transform.setRotation(q);
    // br.sendTransform(tf::StampedTransform(transform, scan->header.stamp, "odom", "lidar_front"));

#if defined(SAVE_DATA_TO_FILE) && SAVE_DATA_TO_FILE
    global_map->addPose(current_pose, Vector3d(odom->pose.pose.position.x, odom->pose.pose.position.y, _rz));
//    fprintf(csv_poses_file, "%d, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f\n",
//        scan->header.stamp.sec,
//        current_pose[0], current_pose[1], current_pose[2],
//        odom->pose.pose.position.x, odom->pose.pose.position.y, _rz);
#endif

    delete current_frame;
    current_frame = new NDTFrame(initial_trans, SIDE_M, SIDE_M, SIDE_M, true);

    scan_loop_rate.sleep();
    matcher_mutex.unlock();
}

int main(int argc, char** argv)
{
    first_iter = true;
    global_trans = Vector3d::Zero();
    previous_pose = Vector3d::Zero();
    trans_estimate = Vector3d::Zero();

    printf("PSO Number of Iterations: %d\n"
           "PSO Population Size: %d\n"
           "NDT Cell Size: %f\n"
           "NDT Window Size: %d\n",
        PSO_ITERATIONS, PSO_POPULATION_SIZE, CELL_SIZE, NDT_WINDOW_SIZE);
    printf("pso global best iter, fitness, x, y, theta, execution time\n");

    ros::init(argc, argv, "ndtpso_slam");
    ros::NodeHandle nh("~");
    std::string param_scan_topic, param_lidar_frame;
    int param_map_size;

    nh.param<std::string>("scan_topic", param_scan_topic, "/scan_front");
    nh.param<std::string>("scan_frame", param_lidar_frame, "lidar_front");
    nh.param("map_size", param_map_size, 25);

    char filename[256];
    sprintf(filename, "%s-%d", param_scan_topic.substr(1).c_str(), ros::Time::now().sec);

#if defined(SAVE_DATA_TO_FILE) && SAVE_DATA_TO_FILE
    global_map = new NDTFrame(Vector3d::Zero(), static_cast<unsigned short>(param_map_size), static_cast<unsigned short>(param_map_size), param_map_size);
// char log_data_filename[256];
// sprintf(log_data_filename, "%s.csv", filename);
// csv_poses_file = fopen(log_data_filename, "w");
// fprintf(csv_poses_file, "t, xP, yP, thP, xO, yO, thO\n");
// ROS_INFO("Saving poses and odometries to \"%s\"", log_data_filename);
#endif

    ROS_INFO("scan_topic:= \"%s\"", param_scan_topic.c_str());
    ROS_INFO("scan_frame:= \"%s\"", param_lidar_frame.c_str());

    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 1);
    // laser_sub = nh.subscribe<sensor_msgs::LaserScan>(laser_scan_topic, 1, &scan_mathcher);
    tf_listener = new tf::TransformListener(ros::Duration(1));

    tf::StampedTransform transform;
    auto _t = ros::Time(0);

    ROS_INFO("Waiting for tf \"base_link\" -> \"%s\"", param_lidar_frame.c_str());

    while (!tf_listener->waitForTransform("base_link", param_lidar_frame, _t, ros::Duration(3)))
        ;
    ROS_INFO("Got a tf \"base_link\" -> \"%s\"", param_lidar_frame.c_str());
    tf_listener->lookupTransform("base_link", param_lidar_frame, _t, transform);

    auto init_trans = transform.getOrigin();
    initial_trans = Vector3d(init_trans.getX(), init_trans.getY(), tf::getYaw(transform.getRotation()));

    current_frame = new NDTFrame(initial_trans, SIDE_M, SIDE_M, CELL_SIZE);

    message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub(nh, param_scan_topic, 1);
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "/odom", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, nav_msgs::Odometry> ApproxSyncPolicy;
    message_filters::Synchronizer<ApproxSyncPolicy> sync(ApproxSyncPolicy(10), laser_sub, odom_sub);

    sync.registerCallback(boost::bind(&scan_mathcher, _1, _2));

    ros::Rate loop_rate(10);

    ROS_INFO("NDT-PSO SLAM started successfuly");

    ros::spin();

#if defined(SAVE_DATA_TO_FILE) && SAVE_DATA_TO_FILE
    //    fclose(csv_poses_file);
    global_map->dumpMap(filename, true, true, true, 200);
    cout << "Map saved to file " << filename << endl;
#endif
}
