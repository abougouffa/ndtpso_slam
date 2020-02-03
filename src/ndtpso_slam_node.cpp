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
#define SYNC_WITH_LASER_TOPIC true
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
#endif

static geometry_msgs::PoseStamped current_pub_pose;
static ros::Publisher pose_pub;
static ros::Subscriber laser_sub;
static tf::TransformListener* tf_listener;

void scan_mathcher(const sensor_msgs::LaserScanConstPtr& scan, const nav_msgs::OdometryConstPtr& odom)
{
#if defined(SAVE_DATA_TO_FILE) && SAVE_DATA_TO_FILE
    static unsigned int iter_num = 0;
#endif
    matcher_mutex.lock();
    Vector3d current_pose;
    auto start = std::chrono::high_resolution_clock::now();

    current_frame->loadLaser(scan->ranges, scan->angle_min, scan->angle_increment, scan->range_max);

    double _, odom_orientation;
    tf::Matrix3x3(tf::Quaternion(
                      odom->pose.pose.orientation.x,
                      odom->pose.pose.orientation.y,
                      odom->pose.pose.orientation.z,
                      odom->pose.pose.orientation.w))
        .getRPY(_, _, odom_orientation);

    if (first_iter) {
        first_iter = false;
        current_pose << odom->pose.pose.position.x, odom->pose.pose.position.y, odom_orientation;
        previous_pose << odom->pose.pose.position.x, odom->pose.pose.position.y, odom_orientation;
    } else {
        current_pose = ref_frame.align(previous_pose, current_frame);
    }

    previous_pose = current_pose;
    // ref_frame.resetPoints();
    ref_frame.update(current_pose, current_frame);

    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish - start;

    ROS_INFO("Matching finished in : %.5fs", elapsed.count());

#if defined(SAVE_DATA_TO_FILE) && SAVE_DATA_TO_FILE
    if (iter_num == 0)
        global_map->update(current_pose, current_frame);
    iter_num = (iter_num + 1) % 10;
    global_map->addPose(scan->header.stamp.toSec(), current_pose, Vector3d(odom->pose.pose.position.x, odom->pose.pose.position.y, odom_orientation));
#endif

    // Publish 'pose', using the same timestamp of the laserscan (use ros::Time::now() !!)
    current_pub_pose.header.stamp = scan->header.stamp;
    current_pub_pose.header.frame_id = "odom"; // we can read it from config
    current_pub_pose.pose.position.x = current_pose.x();
    current_pub_pose.pose.position.y = current_pose.y();
    current_pub_pose.pose.position.z = 0.;
    tf::Quaternion q_ori;
    q_ori.setRPY(0, 0, current_pose.z());
    current_pub_pose.pose.orientation.x = q_ori.getX();
    current_pub_pose.pose.orientation.y = q_ori.getY();
    current_pub_pose.pose.orientation.z = q_ori.getZ();
    current_pub_pose.pose.orientation.w = q_ori.getW();
    pose_pub.publish(current_pub_pose);

    delete current_frame;
    current_frame = new NDTFrame(initial_trans, SIDE_M, SIDE_M, SIDE_M, true);
    matcher_mutex.unlock();
}

int main(int argc, char** argv)
{
    first_iter = true;
    global_trans = Vector3d::Zero();
    previous_pose = Vector3d::Zero();
    trans_estimate = Vector3d::Zero();


    ros::init(argc, argv, "ndtpso_slam");
    ros::NodeHandle nh("~");

    ROS_INFO("NDTPSO Scan Matcher v%s", ndtpso_slam_VERSION);
    ROS_INFO("Config [PSO Number of Iterations: %d]", PSO_ITERATIONS);
    ROS_INFO("Config [PSO Population Size: %d]", PSO_POPULATION_SIZE);
    ROS_INFO("Config [NDT Cell Size: %d]", CELL_SIZE);
    ROS_INFO("Config [NDT Window Size:: %d]", NDT_WINDOW_SIZE);
 
    std::string param_scan_topic, param_odom_topic, param_lidar_frame;
    int param_map_size, param_rate;

    nh.param<std::string>("scan_topic", param_scan_topic, "/scan_front");
    nh.param<std::string>("odom_topic", param_odom_topic, "/odom");
    nh.param<std::string>("scan_frame", param_lidar_frame, "lidar_front");
    nh.param("map_size", param_map_size, 25);
    nh.param("rate", param_rate, 100);

#if defined(SYNC_WITH_LASER_TOPIC) &&  SYNC_WITH_LASER_TOPIC
    std::string param_sync_topic;
    nh.param<std::string>("sync_topic", param_sync_topic, param_scan_topic);
    ROS_INFO("sync_topic:= \"%s\"", param_sync_topic.c_str());
#endif

    ROS_INFO("scan_topic:= \"%s\"", param_scan_topic.c_str());
    ROS_INFO("odom_topic:= \"%s\"", param_odom_topic.c_str());
    ROS_INFO("scan_frame:= \"%s\"", param_lidar_frame.c_str());
    ROS_INFO("rate:= %dHz", param_rate);

#if defined(SAVE_DATA_TO_FILE) && SAVE_DATA_TO_FILE
    global_map = new NDTFrame(Vector3d::Zero(), static_cast<unsigned short>(param_map_size), static_cast<unsigned short>(param_map_size), param_map_size);
#endif

    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 1);
    tf_listener = new tf::TransformListener(ros::Duration(1));
    tf::StampedTransform transform;

    ROS_INFO("Waiting for tf \"base_link\" -> \"%s\"", param_lidar_frame.c_str());
    while (!tf_listener->waitForTransform("base_link", param_lidar_frame, ros::Time(0), ros::Duration(2)))
        ;
    ROS_INFO("Got a tf \"base_link\" -> \"%s\"", param_lidar_frame.c_str());
    tf_listener->lookupTransform("base_link", param_lidar_frame, ros::Time(0), transform);

    auto init_trans = transform.getOrigin();
    initial_trans = Vector3d(init_trans.getX(), init_trans.getY(), tf::getYaw(transform.getRotation()));
    current_frame = new NDTFrame(initial_trans, SIDE_M, SIDE_M, CELL_SIZE);

    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::LaserScan 
        , nav_msgs::Odometry

#if defined(SYNC_WITH_LASER_TOPIC) &&  SYNC_WITH_LASER_TOPIC
        , sensor_msgs::LaserScan
#endif
        > ApproxSyncPolicy;
    
    message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub(nh, param_scan_topic, 1);
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, param_odom_topic, 1);
#if defined(SYNC_WITH_LASER_TOPIC) &&  SYNC_WITH_LASER_TOPIC
    message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_sync(nh, param_sync_topic, 1);
#endif

    message_filters::Synchronizer<ApproxSyncPolicy> 
        sync(ApproxSyncPolicy(10), 
            laser_sub
            , odom_sub
#if defined(SYNC_WITH_LASER_TOPIC) &&  SYNC_WITH_LASER_TOPIC
            , laser_sub_sync
#endif
        );
    
    sync.registerCallback(boost::bind(&scan_mathcher, _1, _2));

    ros::Rate loop_rate(param_rate);

    ROS_INFO("NDT-PSO SLAM started successfuly");

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

#if defined(SAVE_DATA_TO_FILE) && SAVE_DATA_TO_FILE
    char filename[256];
    param_scan_topic = param_scan_topic.replace("/", "-");
    sprintf(filename, "%s-%d", param_scan_topic.substr(1).c_str(), ros::Time::now().sec);
    global_map->dumpMap(filename, true, true, true, 200);
    cout << "Map saved to file " << filename << "[.pose.csv, .map.csv, .png]" << endl;
#endif

}
