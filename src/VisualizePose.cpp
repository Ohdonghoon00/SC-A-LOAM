#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <unistd.h>
#include <stdio.h>
#include <algorithm>
#include <chrono>
#include <sstream>
 
#include <boost/program_options.hpp>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>
#include <pcl/common/point_operators.h>
#include <pcl/common/io.h>
#include <pcl/search/organized.h>
#include <pcl/search/octree.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/filters/voxel_grid.h>
#include<pcl_conversions/pcl_conversions.h>
#include<pcl/io/pcd_io.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <rosbag/bag.h>

#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include <opencv2/core.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <glog/logging.h>




ros::Publisher PubLidarPath;
nav_msgs::Path LidarPath;




Eigen::Matrix4f To44RT(std::vector<float> rot)
{

    cv::Mat R( 1, 3, CV_32FC1);
    R.at<float>(0, 0) = rot[0];
    R.at<float>(0, 1) = rot[1];
    R.at<float>(0, 2) = rot[2];

    // cv::Mat T(1, 3, CV_32FC1);
    // T.at<float>(0, 0) = rot[3];
    // T.at<float>(0, 1) = rot[4];
    // T.at<float>(0, 2) = rot[5];

    cv::Rodrigues(R, R);

    Eigen::Matrix4f RT;
    RT << R.at<float>(0, 0), R.at<float>(0, 1), R.at<float>(0, 2), rot[3],
                R.at<float>(1, 0), R.at<float>(1, 1), R.at<float>(1, 2), rot[4],
                R.at<float>(2, 0), R.at<float>(2, 1), R.at<float>(2, 2), rot[5],
                0,                 0,                   0,                  1;

    return RT;
}

Eigen::Quaternionf GetQuaternion(std::vector<float> KeyframePose)
{
    Eigen::Matrix4f R = To44RT(KeyframePose);
    Eigen::Matrix3f r;
    r <<    R(0, 0), R(0, 1), R(0, 2),
            R(1, 0), R(1, 1), R(1, 2),
            R(2, 0), R(2, 1), R(2, 2);
    
    Eigen::Quaternionf q(r);

    return q;
}







int main(int argc, char **argv) 
{

    ros::init(argc, argv, "Visualize_Pose");
    ros::NodeHandle nh("~");
    
    
    std::string data_dir;
    int publish_delay;
    nh.getParam("data_dir", data_dir);
    nh.getParam("publish_delay", publish_delay);
    

    PubLidarPath = nh.advertise<nav_msgs::Path>("/aft_mapped_path", 100);

    std::string LidarPosePath = "/home/multipleye/Dataset/201014_skt_lobby_day_lidar/LidarOdometryToCameraTimestamp.txt";
    std::ifstream LidarPoseFile(LidarPosePath, std::ifstream::in);

    if(!LidarPoseFile.is_open()){
        std::cout << " Lidar pose txt file failed to open " << std::endl;
        return EXIT_FAILURE;
    }
    
    std::string LidarPoseline;
    int Lidarline_num = 0;
    std::vector<Eigen::Vector3f> LidarPose_r;
    std::vector<Eigen::Vector3f> LidarPose_t;
    
    // publish delay
    ros::Rate rate(10.0 / publish_delay);

    while(std::getline(LidarPoseFile, LidarPoseline) && ros::ok()){

        
        std::string Lidarvalue;
        std::vector<std::string> Lidarvalues;
        std::stringstream ss1(LidarPoseline);
        while(std::getline(ss1, Lidarvalue, ' '))
            Lidarvalues.push_back(Lidarvalue);
        
        Eigen::Vector3f r;
        r << std::stof(Lidarvalues[1]), std::stof(Lidarvalues[2]), std::stof(Lidarvalues[3]); 
        LidarPose_r.push_back(r);

        Eigen::Vector3f t;
        t << std::stof(Lidarvalues[4]), std::stof(Lidarvalues[5]), std::stof(Lidarvalues[6]); 
        LidarPose_t.push_back(t);

        Lidarline_num++;
        
        // publish delay
        ros::Rate rate(10.0 / publish_delay);

        std::vector<float> KeyframePose;
        KeyframePose.resize(6);
        KeyframePose[0] = r(0);
        KeyframePose[1] = r(1);
        KeyframePose[2] = r(2);
        KeyframePose[3] = t(0);
        KeyframePose[4] = t(1);
        KeyframePose[5] = t(2);

    
    
        Eigen::Quaternionf q = GetQuaternion(KeyframePose);
        nav_msgs::Odometry KeyframePoseOutput;
        KeyframePoseOutput.header.frame_id = "/camera_init";
        // KeyframePoseOutput.header.stamp = Keyframe_timestamp;
        KeyframePoseOutput.pose.pose.orientation.x = q.x();
		KeyframePoseOutput.pose.pose.orientation.y = q.y();
		KeyframePoseOutput.pose.pose.orientation.z = q.z();
		KeyframePoseOutput.pose.pose.orientation.w = q.w();
		KeyframePoseOutput.pose.pose.position.x = KeyframePose[3];
		KeyframePoseOutput.pose.pose.position.y = KeyframePose[4];
		KeyframePoseOutput.pose.pose.position.z = KeyframePose[5];
		// pubKeyframeOdom.publish(KeyframePoseOutput);
        
        geometry_msgs::PoseStamped Keyframepose;
	    Keyframepose.header = KeyframePoseOutput.header;
		Keyframepose.pose = KeyframePoseOutput.pose.pose;

		// LidarPath.header.stamp = KeyframePoseOutput.header.stamp;
		LidarPath.header.frame_id = "/camera_init";
		LidarPath.poses.push_back(Keyframepose);
		PubLidarPath.publish(LidarPath);
        
        rate.sleep();
    }
        
    
    





    LidarPoseFile.close();



    return 0;
}