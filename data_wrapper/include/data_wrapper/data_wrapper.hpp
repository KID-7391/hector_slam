#ifndef DATA_WRAPPER_HPP
#define DATA_WRAPPER_HPP
#include "data_wrapper/scan_odom.h"
#include <ros/ros.h>
#include <cstring>

class ScanOdom{
    data_wrapper::scan_odom data;
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber scan_sub, odom_sub;
    ros::Rate *rate_ptr;
    ros::MultiThreadedSpinner *spinner;
    bool ready_scan, ready_odom;
public:
    ScanOdom(const char *scan_topic, const char *odom_topic, double frequency);
    void CallScan(const sensor_msgs::LaserScan::ConstPtr &msg);
    void CallOdom(const nav_msgs::Odometry::ConstPtr &msg);
    void run();
};

#endif