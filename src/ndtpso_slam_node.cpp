#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "ndtpso_slam/ndtcell.h"
#include "ndtpso_slam/ndtframe.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/Float32.h"
#include <chrono>
#include <cstdio>
#include <eigen3/Eigen/Core>
#include <iostream>
#include <laser_geometry/laser_geometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <mutex>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

using namespace Eigen;
using std::cout;
using std::endl;

static tf::TransformListener* tf_listener;

static FILE* log_data;

#define CELL_SIZE .5
#define SIDE_M 200

static std::mutex matcher_mutex;
static NDTFrame ref_frame(Vector3d::Zero(), 50, 50, CELL_SIZE);
;
static NDTFrame* current_frame; //(Vector3d(0.650, 0.440, 0.820), SIDE_M, SIDE_M, CELL_SIZE);
static NDTFrame global_map(Vector3d::Zero(), 25, 25, CELL_SIZE);
static Vector3d global_trans, previous_pose, trans_estimate, initial_trans;
static bool first_iter;
static geometry_msgs::PoseStamped current_pub_pose, pose2;

//laser_geometry::LaserProjection projector_;
//tf::TransformListener tfListener_;
static ros::Publisher pose_pub;
//static ros::Publisher laser_pub;
static ros::Subscriber laser_sub;

//ros::Publisher point_cloud_publisher_;

void scan_mathcher(const sensor_msgs::LaserScanConstPtr& scan, const nav_msgs::OdometryConstPtr& odom)
{
    static unsigned int iter_num = 0;
    matcher_mutex.lock();
    ros::Rate scan_loop_rate(10);
    auto start = std::chrono::high_resolution_clock::now();

    current_frame->loadLaser(scan->ranges, scan->angle_min, scan->angle_increment, scan->range_max);

    Vector3d current_pose;

    double _, _rz;
    tf::Matrix3x3(tf::Quaternion(
                      odom->pose.pose.orientation.x,
                      odom->pose.pose.orientation.y,
                      odom->pose.pose.orientation.z,
                      odom->pose.pose.orientation.w))
        .getRPY(_, _, _rz);

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
    printf("%.5f\n", elapsed.count());

    if (iter_num == 0)
        global_map.update(current_pose, current_frame);

    iter_num = (iter_num + 1) % 10;

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
    //    /tf_listener->transformPose("base_link", current_pub_pose, pose2);
    pose_pub.publish(current_pub_pose);

    //    sensor_msgs::LaserScan scan2 = *scan;
    //    scan2.angle_increment = scan->angle_increment;
    //    scan2.angle_max = scan->angle_max;
    //    scan2.angle_min = scan->angle_min;
    //    scan2.range_max = scan->range_max;
    //    scan2.range_min = scan->range_min;
    //    scan2.scan_time = scan->scan_time;
    //    scan2.time_increment = scan->time_increment;
    //    scan2.header.frame_id = "laser";
    //    scan2.intensities = scan->intensities;
    //    scan2.ranges = scan->ranges;
    //    scan2.header.seq = seq++;
    //    scan2.header.stamp = ros::Time::now();
    //    laser_pub.publish(scan2);

    //static tf::TransformBroadcaster br;
    //    geometry_msgs::TransformStamped transformStamped;

    //    tf::Transform transform;
    //    transform.setOrigin(tf::Vector3(current_pose[0], current_pose[1], 0.0));
    //    tf::Quaternion q;
    //    q.setRPY(0, 0, current_pose[2]);
    //    transform.setRotation(q);
    //    br.sendTransform(tf::StampedTransform(transform, scan->header.stamp, "odom", "lidar_front"));

    //    sensor_msgs::PointCloud2 cloud;
    //    projector_.transformLaserScanToPointCloud("base_link", *scan, cloud, tfListener_);
    //    point_cloud_publisher_.publish(cloud);

    global_map.addPose(current_pose, Vector3d(odom->pose.pose.position.x, odom->pose.pose.position.y, 0.));
    fprintf(log_data, "%d, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f\n",
        scan->header.stamp.sec,
        current_pose[0], current_pose[1], current_pose[2],
        odom->pose.pose.position.x, odom->pose.pose.position.y, _rz);

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

    nh.param<std::string>("scan_topic", param_scan_topic, "/scan_front");
    nh.param<std::string>("scan_frame", param_lidar_frame, "lidar_front");

    char filename[256], log_data_filename[256];
    sprintf(filename, "%s-%d", param_scan_topic.substr(1).c_str(), ros::Time::now().sec);
    sprintf(log_data_filename, "%s.csv", filename);
    log_data = fopen(log_data_filename, "w");
    fprintf(log_data, "t, xP, yP, thP, xO, yO, thO\n");

    ROS_INFO("Saving poses and odometries to \"%s\"", log_data_filename);
    ROS_INFO("scan_topic:= %s", param_scan_topic.c_str());
    ROS_INFO("scan_frame:= %s", param_lidar_frame.c_str());

    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 1);
    //laser_sub = nh.subscribe<sensor_msgs::LaserScan>(laser_scan_topic, 1, &scan_mathcher);
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
    //message_filters::Synchronizer<ApproxSyncPolicy> sync(ApproxSyncPolicy(10), laser_sub, odom_sub);
    message_filters::TimeSynchronizer<sensor_msgs::LaserScan, nav_msgs::Odometry> sync(laser_sub, odom_sub, 10);

    sync.registerCallback(boost::bind(&scan_mathcher, _1, _2));

    ros::Rate loop_rate(10);

    ROS_INFO("NDT-PSO SLAM launched successfuly");

    ros::spin();

    fclose(log_data);
    global_map.saveImage(filename, 250);
    cout << "Map saved to file " << filename << endl;
}
