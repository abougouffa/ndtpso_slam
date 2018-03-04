#include "geometry_msgs/PoseStamped.h"
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
#include <mutex>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

using namespace Eigen;
using std::cout;
using std::endl;

unsigned long sec;

#define CELL_SIZE 1.
//#define CELL_SIZE_SMALL .4

static std::mutex matcher_mutex;
static NDTFrame ref_frame(Vector3d::Zero(), 200, 200, CELL_SIZE);
static NDTFrame current_frame(Vector3d::Zero(), 80, 160, 80, true);
static NDTFrame global_map(Vector3d::Zero(), 200, 200, CELL_SIZE);
static Vector3d global_trans, previous_trans, trans_estimate;
static bool first_iter;
static geometry_msgs::PoseStamped current_pose;

//laser_geometry::LaserProjection projector_;
//tf::TransformListener tfListener_;
ros::Publisher pose_pub;
ros::Publisher laser_pub;
ros::Subscriber laser_sub;

//ros::Publisher point_cloud_publisher_;

#ifdef CELL_SIZE_SMALL
static NDTFrame ref_frame_small(Vector3d::Zero(), 200, 200, CELL_SIZE_SMALL);
static NDTFrame current_frame_small(Vector3d::Zero(), 100, 100, CELL_SIZE_SMALL);
#endif

void scan_mathcher(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    static unsigned int iter_num = 0;
    matcher_mutex.lock();
    //    ros::Rate scan_loop_rate(2);
    auto start = std::chrono::high_resolution_clock::now();

    current_frame.loadLaser(scan->ranges, scan->angle_min, scan->angle_max, scan->angle_increment, scan->range_max);

#ifdef CELL_SIZE_SMALL
    current_frame_small.loadLaser(scan->ranges, scan->angle_min, scan->angle_max, scan->angle_increment);
#endif

    Vector3d current_trans;

    if (first_iter) {
        first_iter = false;
        current_trans << .0, .0, .0;
        previous_trans << .0, .0, .0;
    } else {
        current_trans = ref_frame.align(previous_trans, &current_frame);
//        for (unsigned char p = 0; p < 2; ++p)
//            if ((current_trans[p] > (previous_trans[p] + 3)) || (current_trans[p] < (previous_trans[p] - 3))) {
//                current_frame = NDTFrame(Vector3d::Zero(), 100, 100, 100);
//                ROS_INFO("Unaccepted Pose\n");
//                matcher_mutex.unlock();
//                return;
//            }
#ifdef CELL_SIZE_SMALL
        current_trans = ref_frame_small.align(current_trans, &current_frame_small);
#endif
    }

    previous_trans = current_trans;

    ref_frame.resetPoints();
    ref_frame.update(current_trans, &current_frame);

#ifdef CELL_SIZE_SMALL
    ref_frame_small.resetPoints();
    ref_frame_small.update(current_trans, &current_frame_small);
#endif
    auto finish = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> elapsed = finish - start;

    //    std::cout << endl
    //              << "Elapsed time: " << elapsed.count() << " s\n";
    printf("%.5f\n", elapsed.count());

    if (iter_num == 0)
        global_map.update(current_trans, &current_frame);

    iter_num = (iter_num + 1) % 15;

    /*
    current_pose.header.stamp = ros::Time::now();
    current_pose.header.frame_id = "odom";
    current_pose.pose.position.x = current_trans[0];
    current_pose.pose.position.y = current_trans[1];
    current_pose.pose.position.z = 0;
    current_pose.pose.orientation.x = 0;
    current_pose.pose.orientation.y = 0;
    current_pose.pose.orientation.z = sin(current_trans[2] / 2.);
    current_pose.pose.orientation.w = cos(current_trans[2] / 2.);

    sensor_msgs::LaserScan scan2 = *scan;
    scan2.angle_increment = scan->angle_increment;
    scan2.angle_max = scan->angle_max;
    scan2.angle_min = scan->angle_min;
    scan2.range_max = scan->range_max;
    scan2.range_min = scan->range_min;
    scan2.scan_time = scan->scan_time;
    scan2.time_increment = scan->time_increment;
    scan2.header.frame_id = "laser";
    scan2.intensities = scan->intensities;
    scan2.ranges = scan->ranges;
    scan2.header.seq = sec++;
    scan2.header.stamp = ros::Time::now();
    pose_pub.publish(current_pose);
    laser_pub.publish(scan2);

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(current_trans[0], current_trans[1], 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, current_trans[2]);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "laser"));
    */

    //    sensor_msgs::PointCloud2 cloud;
    //    projector_.transformLaserScanToPointCloud("base_link", *scan, cloud, tfListener_);
    //    point_cloud_publisher_.publish(cloud);

    global_map.addPose(current_trans);

    current_frame = NDTFrame(Vector3d::Zero(), 80, 80, 80, true);
#ifdef CELL_SIZE_SMALL
    current_frame_small = NDTFrame(Vector3d::Zero(), 20, 20, CELL_SIZE_SMALL);
#endif
    //    scan_loop_rate.sleep();
    matcher_mutex.unlock();
}

int main(int argc, char** argv)
{
    first_iter = true;
    sec = 0;
    global_trans = Vector3d::Zero();
    previous_trans = Vector3d::Zero();
    trans_estimate = Vector3d::Zero();

    printf("PSO Number of Iterations: %d\n"
           "PSO Population Size: %d\n"
           "NDT Cell Size: %f\n"
           "NDT Window Size: %d\n",
        PSO_ITERATIONS, PSO_POPULATION_SIZE, CELL_SIZE, NDT_WINDOW_SIZE);
    printf("pso global best iter, fitness, x, y, theta, execution time\n");

    ros::init(argc, argv, "ndtpso_slam");
    ros::NodeHandle nh;
    //    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("ndtpso/pose", 1);
    //    laser_pub = nh.advertise<sensor_msgs::LaserScan>("ndtpso/scan", 10);
    laser_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1, &scan_mathcher);
    ros::Rate loop_rate(10);

    //    ROS_INFO("NDT-PSO SLAM launched successfuly");

    while (ros::ok()) {
        ros::spin();
        //        pose_pub.publish(current_pose);
        //        loop_rate.sleep();
    }

    //    ref_frame.saveImage("globalmap-ALL", 80);

    char filename[100];
    sprintf(filename, "globalmap-08%X", ros::Time::now().sec);
    global_map.saveImage(filename, 25);
    //    printf("Map saved to file %s\n", filename);
}
