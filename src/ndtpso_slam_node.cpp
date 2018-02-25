#include "geometry_msgs/PoseStamped.h"
#include "ndtpso_slam/ndtcell.h"
#include "ndtpso_slam/ndtframe.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float32.h"
#include <chrono>
#include <eigen3/Eigen/Core>
#include <iostream>
#include <mutex>
#include <stdio.h>

using namespace Eigen;
using std::cout;
using std::endl;

static std::mutex matcher_mutex;
static NDTFrame ref_frame(Vector3d::Zero(), 200, 200, 1.);
static NDTFrame global_map(Vector3d::Zero(), 200, 200, 1.);
static NDTFrame current_frame(Vector3d::Zero(), 20, 20, 1.);
static Vector3d global_trans, previous_trans, trans_estimate;
static bool first_iter;
static geometry_msgs::PoseStamped current_pose;

void scan_mathcher(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    static unsigned int iter_num = 0;
    matcher_mutex.lock();
    ros::Rate scan_loop_rate(2);
    auto start = std::chrono::high_resolution_clock::now();

    current_frame.loadLaser(scan->ranges, scan->angle_min, scan->angle_max, scan->angle_increment);

    Vector3d current_trans;

    if (first_iter) {
        first_iter = false;
        current_trans << .0, .0, .0;
        previous_trans << .0, .0, .0;
    } else {
        current_trans = ref_frame.align(previous_trans, &current_frame);
    }

    previous_trans = current_trans;

    ref_frame.resetPoints();
    ref_frame.update(current_trans, &current_frame);

    if (iter_num == 0)
        global_map.update(current_trans, &current_frame);

    global_map.addPose(current_trans);

    iter_num = (iter_num + 1) % 20;

    //    char filename[200];
    //    sprintf(filename, "globalmap-multilayers-%d", iter_num);
    //    global_map.saveImage(filename, 100);

    auto finish = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> elapsed = finish - start;

    std::cout << endl
              << "Elapsed time: " << elapsed.count() << " s\n";

    current_frame = NDTFrame(Vector3d::Zero(), 20, 20, 1.);
    //    scan_loop_rate.sleep();
    matcher_mutex.unlock();
}

int main(int argc, char** argv)
{
    first_iter = true;
    global_trans = Vector3d::Zero();
    previous_trans = Vector3d::Zero();
    trans_estimate = Vector3d::Zero();

    ros::init(argc, argv, "ndtpso_slam");
    ros::NodeHandle nh;
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("ndtpso/pose", 1);
    ros::Subscriber laser_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1, &scan_mathcher);
    ros::Rate loop_rate(10);

    while (ros::ok()) {
        ros::spinOnce();
        pose_pub.publish(current_pose);
        loop_rate.sleep();
    }

    //    ref_frame.saveImage("globalmap-ALL", 80);
    global_map.saveImage("globalmap-20IT", 20);
}
