#include "ros/ros.h"
#include "data_wrapper/data_wrapper.hpp"

ScanOdom::ScanOdom(const char *scan_topic, const char *odom_topic, double frequency){
    pub = nh.advertise<data_wrapper::scan_odom>("scan_odom", 1, 52428800);
    scan_sub = nh.subscribe(scan_topic, 1, &ScanOdom::CallScan, this);
    odom_sub = nh.subscribe(odom_topic, 1, &ScanOdom::CallOdom, this);
    spinner = new ros::MultiThreadedSpinner(3);
    rate_ptr = new ros::Rate(frequency);
}

void ScanOdom::CallScan(const sensor_msgs::LaserScan::ConstPtr &msg){data.scan = *msg;ready_scan = true;}
void ScanOdom::CallOdom(const nav_msgs::Odometry::ConstPtr &msg){
    data.odom = *msg;
    ready_odom = true;
    if(ready_odom && ready_scan){
        pub.publish(data);
    }
    rate_ptr->sleep();
}

void ScanOdom::run(){
    spinner->spin();
}

int main(int argc, char **argv){
    ros::init(argc, argv, "data_wrapper");

    ScanOdom *so = new ScanOdom("scan", "odom", 30);

    so->run();

    return 0;
}
