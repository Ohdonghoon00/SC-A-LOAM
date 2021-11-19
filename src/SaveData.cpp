#include <fstream>
#include <iostream>
#include <string>
#include "opencv2/opencv.hpp"
#include <opencv2/core.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <rosbag/bag.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

std::queue<nav_msgs::Odometry::ConstPtr> OdometryBuf;
ros::Subscriber SubLidarOdometry;
std::string SaveOdometry = "/home/multipleye/Dataset/201014_skt_lobby_day_lidar/LidarPoseAftPgo.txt";
std::ofstream file(SaveOdometry);

Eigen::Vector3f QuaternionTo3Vec(Eigen::Quaternionf q)
{
    Eigen::Matrix3f rot = q.normalized().toRotationMatrix();
    
    float data[] = { rot(0, 0), rot(0, 1), rot(0, 2),
                    rot(1, 0), rot(1, 1), rot(1, 2),
                    rot(2, 0), rot(2, 1), rot(2, 2)};
    
    cv::Mat r(3, 3, CV_32FC1, data);
    cv::Rodrigues(r, r);

    Eigen::Vector3f R;
    R << r.at<float>(0, 0), r.at<float>(1, 0), r.at<float>(2, 0);

    return R;
}

void SaveOdometryData()
{

    
    
    while(!OdometryBuf.empty()){
        float x = OdometryBuf.front()->pose.pose.position.x;
        float y = OdometryBuf.front()->pose.pose.position.y;
        float z = OdometryBuf.front()->pose.pose.position.z;
        
        Eigen::Quaternionf q;
        q.x() = OdometryBuf.front()->pose.pose.orientation.x;
        q.y() = OdometryBuf.front()->pose.pose.orientation.y;
        q.z() = OdometryBuf.front()->pose.pose.orientation.z;
        q.w() = OdometryBuf.front()->pose.pose.orientation.w;
        
        Eigen::Vector3f Ori = QuaternionTo3Vec(q);
        file << Ori(0) << " " << Ori(1) << " " << Ori(2) << " " << x << " " << y << " " << z << " " << std::endl;
        std::cout << OdometryBuf.front()->header.stamp << std::endl;
        OdometryBuf.pop();
    }

    

    
}
void LidarOdometryHandler(const nav_msgs::Odometry::ConstPtr &Odometry)
{
    OdometryBuf.push(Odometry);
    SaveOdometryData();
}


int main(int argc, char** argv)
{
    
    ros::init(argc, argv, "Save_Data");
    ros::NodeHandle nh("~");
    SubLidarOdometry = nh.subscribe<nav_msgs::Odometry>("/aft_pgo_odom", 100, LidarOdometryHandler);
    
    
    // std::string data_dir; 
    // ros::param::get("data_dir", data_dir);
    // SaveOdometry = data_dir + "LidarOdometryToCameraTimestamp.txt";
    // file = SaveOdometry;


    

    while(ros::ok()){
        ros::Rate rate(5);
        ros::spin();
        rate.sleep();
    }


    return 0;
}