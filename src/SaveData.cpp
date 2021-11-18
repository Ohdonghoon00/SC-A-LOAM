#include <fstream>
#include <iostream>
#include <string>
#include "opencv2/opencv.hpp"
#include <opencv2/core.hpp>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <rosbag/bag.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>


ros::Subscriber SubLidarPath;

int main(int argc, char** argv)
{
    
    ros::init(argc, argv, "Save_Data");
    ros::NodeHandle nh("~");
    
    SubLidarPath = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_cloud_registered_local", 100, laserCloudFullResHandler);

    // launch parameter
    std::string data_dir;
    int publish_delay;
    
    nh.getParam("data_dir", data_dir);
    nh.getParam("publish_delay", publish_delay);


    std::string SavePath = data_dir;
    std::ofstream file(SavePath);


    return 0;
}