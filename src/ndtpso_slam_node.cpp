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

using namespace Eigen;
using std::cout;
using std::endl;

static std::mutex matcher_mutex;
static NdtFrame ref_frame(Vector3d::Zero(), 20, 20, 1.5);
static NdtFrame global_map(Vector3d::Zero(), 200, 200, 1.5);
static NdtFrame current_frame(Vector3d::Zero(), 20, 20, 1.5);
static Vector3d global_trans, previous_trans, trans_estimate;
static unsigned int iter_num;
static geometry_msgs::PoseStamped current_pose;

void scan_mathcher(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    matcher_mutex.lock();

    //    static char iter_counter = 0;

    //    if (iter_counter++ < 5) {
    //        current_frame.loadLaser(scan->ranges, scan->angle_min, scan->angle_max);
    //    } else {
    //        iter_counter = 0;

    //    ros::Rate loop_rate(1);

    auto start = std::chrono::high_resolution_clock::now();

    current_frame.loadLaser(scan->ranges, scan->angle_min, scan->angle_max);

    ref_frame.build();
    Vector3d current_trans;

    if (iter_num == 0)
        current_trans << .0, .0, .0;
    else
        current_trans = ref_frame.align(trans_estimate, &current_frame);

    global_trans += current_trans;

    current_pose.header.stamp = scan->header.stamp;
    current_pose.pose.position.x = global_trans[0];
    current_pose.pose.position.y = global_trans[1];
    current_pose.pose.orientation.w = global_trans[2];

    trans_estimate = current_trans / 2.;
    previous_trans = current_trans;
    ref_frame = current_frame;

    global_map.update(global_trans, &current_frame);
    ++iter_num;

    auto finish = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> elapsed = finish - start;

    std::cout << endl
              << "Elapsed time: " << elapsed.count() << " s\n";

    current_frame = NdtFrame(Vector3d::Zero(), 40, 40, 1.);
    //    loop_rate.sleep();
    //    }
    matcher_mutex.unlock();
}

int main(int argc, char** argv)
{
    iter_num = 0;
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

    global_map.print();
}
