


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





ros::Publisher LaserCloudNoDistortion, KeyframeLaserCloud, pubKeyframePath;
nav_msgs::Path KeyframePath;

const std::vector<float> imu2rig_pose = {-0.011773881878296,-2.212344247385963,2.229193892963689,-0.016975989407230,0.016444757006134,0.128779023189435};
const std::vector<float> lidar2rig_pose = {1.5620435019860173, -0.005377623186353324, 0.003014408980859652, -8.458334129298635E-4, -0.19542397891778734, -0.0012719333618026098};

struct LidarData 
{

    std::vector<char> binary_data;
    int64_t timestamp_ns;
    int num_points, num_blocks;
    uint8_t num_channels;

    
    LidarData()
        : num_points(0), num_blocks(0), num_channels(0) {}
    virtual ~LidarData() {}
    
    
    float* points_ptr() const { return (float*) binary_data.data(); }
    uint8_t* intensities_ptr() const { return (uint8_t*) &binary_data[3 * num_points * sizeof(float)]; } // reflectivity
    uint8_t* azimuth_idxs_ptr() const { return intensities_ptr() + num_points; }
    float* azimuth_degs_ptr() const { return (float*) (azimuth_idxs_ptr() + num_points); }
    
     

};

// std::ifstream ReadData(std::string DataPath)
// {
//     std::ifstream DataFile(DataPath, std::ifstream::in);

//     if(!DataFile.is_open()){
//         std::cout << " Data file failed to open " << std::endl;
//         return EXIT_FAILURE;
//     }

//     return DataFile;
// }


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

Eigen::Matrix4f gyroToRotation(std::vector<float> gyro)
{
    float t = 0.005; // 200Hz
    float angle_x(gyro[0] * t), angle_y(gyro[1] * t), angle_z(gyro[2] * t);  
    
    float data_x[] = {  1.0, 0.0, 0.0,
                        0.0, cos(angle_x), sin(angle_x),
                        0.0, -sin(angle_x), cos(angle_x)};

    float data_y[] = {  cos(angle_y), 0.0, -sin(angle_y),
                        0.0, 1.0, 0.0,
                        sin(angle_y), 0.0, cos(angle_y)};

    float data_z[] = {  cos(angle_z), sin(angle_z), 0.0,
                        -sin(angle_z), cos(angle_z), 0.0,
                        0.0, 0.0, 1.0};

    cv::Mat R_x( 3, 3, CV_32FC1, data_x);
    cv::Mat R_y( 3, 3, CV_32FC1, data_y);
    cv::Mat R_z( 3, 3, CV_32FC1, data_z);
    cv::Mat R = R_x * R_y * R_z;




    Eigen::Matrix4f RT;
    RT << R.at<float>(0, 0), R.at<float>(0, 1), R.at<float>(0, 2), 0,
                R.at<float>(1, 0), R.at<float>(1, 1), R.at<float>(1, 2), 0,
                R.at<float>(2, 0), R.at<float>(2, 1), R.at<float>(2, 2), 0,
                0,                  0,                  0,              1;

    return RT;
}

float ToAngle(Eigen::Matrix4f LidarRotation)
{
    float data[] = {    LidarRotation(0, 0), LidarRotation(0, 1), LidarRotation(0, 2),
                        LidarRotation(1, 0), LidarRotation(1, 1), LidarRotation(1, 2),
                        LidarRotation(2, 0), LidarRotation(2, 1), LidarRotation(2, 2)};

    
    cv::Mat rot(3, 3, CV_32FC1, data);
    // cv::Mat r;
    cv::Rodrigues(rot, rot);
    float angle = sqrt( rot.at<float>(0, 0) * rot.at<float>(0, 0) + 
                        rot.at<float>(1, 0) * rot.at<float>(1, 0) +
                        rot.at<float>(2, 0) * rot.at<float>(2, 0) );

    return angle;
}

Eigen::Vector3f ToAxis(Eigen::Matrix4f LidarRotation)
{
    float angle = ToAngle(LidarRotation);
    float data[] = {    LidarRotation(0, 0), LidarRotation(0, 1), LidarRotation(0, 2),
                        LidarRotation(1, 0), LidarRotation(1, 1), LidarRotation(1, 2),
                        LidarRotation(2, 0), LidarRotation(2, 1), LidarRotation(2, 2)};

    
    cv::Mat rot(3, 3, CV_32FC1, data);
    // cv::Mat r;
    cv::Rodrigues(rot, rot);

    Eigen::Vector3f Axis;
    Axis << rot.at<float>(0, 0), rot.at<float>(1, 0), rot.at<float>(2, 0);
    Axis = Axis / angle;

    return Axis;

}

void MoveDistortionPoints(pcl::PointCloud<pcl::PointXYZ> &points, Eigen::Matrix4f LidarRotation, int ScanStepNum, int num_seqs)
{
    int PointNum = points.size();
    Eigen::MatrixXf MatrixPoints(4, PointNum);
    for(int i = 0; i < PointNum; i++){
        MatrixPoints(0, i) = points[i].x;
        MatrixPoints(1, i) = points[i].y;
        MatrixPoints(2, i) = points[i].z;
        MatrixPoints(3, i) = 1.0;
    }
    
    float angle = ToAngle(LidarRotation);
    Eigen::Vector3f Axis = ToAxis(LidarRotation);
    
    float ratio = (((float)(ScanStepNum + 1) / (float)num_seqs) * angle);
    Axis = Axis * ratio;
    
    float data[] = {Axis(0, 0), Axis(1, 0), Axis(2, 0)};
    cv::Mat R(3, 1, CV_32FC1, data);
    // cv::Mat r;
    cv::Rodrigues(R, R);

    Eigen::Matrix4f RT;
    RT <<   R.at<float>(0, 0), R.at<float>(0, 1), R.at<float>(0, 2), 0,
            R.at<float>(1, 0), R.at<float>(1, 1), R.at<float>(1, 2), 0,
            R.at<float>(2, 0), R.at<float>(2, 1), R.at<float>(2, 2), 0,
            0,                  0,                  0,              1;


    Eigen::Matrix4Xf MatrixPoints_ = RT.inverse() * MatrixPoints;
    points.clear();
    for(int i = 0; i < PointNum; i++){
        pcl::PointXYZ Point;
        Point.x = MatrixPoints_(0, i);
        Point.y = MatrixPoints_(1, i);
        Point.z = MatrixPoints_(2, i);
        points.push_back(Point);
    }



}

void MovePointsToKeyframe(pcl::PointCloud<pcl::PointXYZ> &points, std::vector<float> KeyframePose)
{
    int PointNum = points.size();

    Eigen::MatrixXf MatrixPoints(4, PointNum);
    for(int i = 0; i < PointNum; i++){
        MatrixPoints(0, i) = points[i].x;
        MatrixPoints(1, i) = points[i].y;
        MatrixPoints(2, i) = points[i].z;
        MatrixPoints(3, i) = 1.0;
    }

    Eigen::Matrix4Xf MatrixPoints_ = To44RT(KeyframePose) * To44RT(lidar2rig_pose) * MatrixPoints;

    points.clear();
    for(int i = 0; i < PointNum; i++){
        pcl::PointXYZ Point;
        Point.x = MatrixPoints_(0, i);
        Point.y = MatrixPoints_(1, i);
        Point.z = MatrixPoints_(2, i);
        points.push_back(Point);
    }

}

int main(int argc, char **argv) 
{

    ros::init(argc, argv, "Visualize_Keyframe_Pointcloud");
    ros::NodeHandle nh("~");

    LidarData lidar_data;
    
    
    std::string data_dir;
    int publish_delay;
    nh.getParam("data_dir", data_dir);
    nh.getParam("publish_delay", publish_delay);
    
    LaserCloudNoDistortion = nh.advertise<sensor_msgs::PointCloud2>("/undistortion_map", 100);
    KeyframeLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/keyframe_map", 100);
	// pubKeyframeOdom = nh.advertise<nav_msgs::Odometry>("/aft_mapped_path", 100);
    pubKeyframePath = nh.advertise<nav_msgs::Path>("/aft_mapped_path", 100);


    // publish delay
    ros::Rate r(10.0 / publish_delay);
    ros::Rate r_(1);
    const Eigen::Matrix4f RigToIMU = To44RT(imu2rig_pose);
    const Eigen::Matrix4f RigToLidar = To44RT(lidar2rig_pose);

    // Read Data File
    // std::string LidarcsvPath = data_dir + "lidar_timestamp.csv";
    // std::string IMUcsvPath = data_dir + "imu_data.csv";
    // std::string SlamPosePath = data_dir + "slam_poses.txt";

    // std::ifstream LidarcsvFile = ReadData(LidarcsvPath);
    // std::ifstream IMUcsvFile = ReadData(IMUcsvPath);
    // std::ifstream SlamPoseFile = ReadData(SlamPosePath);
    
    // Lidar timestamp.csv path
    std::string LidarcsvPath = data_dir + "lidar_timestamp.csv";
    std::ifstream LidarcsvFile(LidarcsvPath, std::ifstream::in);

    if(!LidarcsvFile.is_open())
    {
        std::cout << " Lidar csv file failed to open " << std::endl;
        return EXIT_FAILURE;
    }
     
    // IMU csv data path
    std::string IMUcsvPath = data_dir + "imu_data.csv";
    std::ifstream IMUcsvFile(IMUcsvPath, std::ifstream::in);

    if(!IMUcsvFile.is_open()){
        std::cout << " IMU csv file failed to open " << std::endl;
        return EXIT_FAILURE;
    }

    // Read SLAM pose.txt
    std::string SlamPosePath = data_dir + "slam_poses.txt";
    std::ifstream SlamPoseFile(SlamPosePath, std::ifstream::in);

    if(!SlamPoseFile.is_open()){
        std::cout << " SLAM pose txt file failed to open " << std::endl;
        return EXIT_FAILURE;
    }


    Eigen::Matrix4f LidarRotation;
    std::string Lidarcsvline;
    std::string SlamPoseline;
    int Lidarline_num = 0;
    int IMUline_num = 0;
    int KeyframePoseline_num = 0;
    Eigen::Matrix4f IMURotation_integral = Eigen::Matrix4f::Identity();
    pcl::PointCloud<pcl::PointXYZ> VisualizeKeyframeLidarPoints;


    // Read Keyframe Pose txt file
    while(std::getline(SlamPoseFile, SlamPoseline) && ros::ok()){
        
        std::cout << " Keyframe Pose line num : " << KeyframePoseline_num << std::endl;
        ros::Time Keyframe_timestamp;

        std::string Cameravalue;
        std::vector<std::string> Cameravalues;

        // Cameravalues[0] -> Camera Fidx
        // Cameravalues[1], [2], [3] -> Rotation
        // Cameravalues[4], [5], [6] -> Translation
        // Cameravalues[7] -> Timestamp

        std::stringstream ss1(SlamPoseline);
        while(std::getline(ss1, Cameravalue, ' '))
            Cameravalues.push_back(Cameravalue);
        double KeyframeTimestamp = std::stod(Cameravalues[7]);
        std::vector<float> KeyframePose = { std::stof(Cameravalues[1]), std::stof(Cameravalues[2]), std::stof(Cameravalues[3]),
                                            std::stof(Cameravalues[4]), std::stof(Cameravalues[5]), std::stof(Cameravalues[6]) };
        // std::cout <<  "keyfrmae timestamp : " << KeyframeTimestamp << std::endl;   

        // read timestamp.csv
        while(std::getline(LidarcsvFile, Lidarcsvline) && ros::ok()){
            // std::cout << "Lidar line num : " << Lidarline_num << "th " << std::endl;

            if(Lidarline_num == 0) {
                Lidarline_num++;
                continue;
            }
            std::string Lidarvalue;
            std::vector<std::string> Lidarvalues;
            
            // Lidarvalues[0] -> First Seq Timestamp (ns)
            // Lidarvalues[1] -> Last Seq Timestamp (ns)
            // Lidarvalues[2] -> Fidx
            // Lidarvalues[3] -> Num pts
            // Lidarvalues[4] -> Date
            
            std::stringstream ss2(Lidarcsvline);
            while(std::getline(ss2, Lidarvalue, ','))
                Lidarvalues.push_back(Lidarvalue);
            int fidx = std::stoi(Lidarvalues[2]);
            double LastScanTimestamp = std::stod(Lidarvalues[1]);
            
            // binary data path
            std::stringstream Lidar_binary_path;
            Lidar_binary_path << data_dir + "lidar/" << std::setfill('0') << std::setw(5) << fidx << ".xyz";
            
            std::ifstream ifs(Lidar_binary_path.str(), std::ifstream::in);
            
            if (!ifs.is_open()) {
                std::cout << "xyz file failed to open: " << std::endl;
                return EXIT_FAILURE;
            }        

            
            const size_t kMaxNumberOfPoints = 1e6; 
            
            
            // std::cout << " Lidar File number : " << fidx << "     " << std::endl;
            
            // read binary data file
            int num_seqs = 0;
            ifs.read((char*)&num_seqs, sizeof(int));
            // std::cout << " num_seqs : " << num_seqs << std::endl;




            std::string IMUcsvline;
            while(std::getline(IMUcsvFile, IMUcsvline) && ros::ok()){
                
                if(IMUline_num == 0) {
                    IMUline_num++;
                    continue;
                }
                
                std::string IMUvalue;
                std::vector<std::string> IMUvalues;
                
                // IMUvalues[0] -> Timestamp (ns)
                // IMUvalues[1] -> Gyro_x
                // IMUvalues[2] -> Gyro_y
                // IMUvalues[3] -> Gyro_z
                // IMUvalues[4] -> Acc_x
                // IMUvalues[5] -> Acc_y
                // IMUvalues[6] -> Acc_z

                std::stringstream ss3(IMUcsvline);
                while(std::getline(ss3, IMUvalue, ','))
                    IMUvalues.push_back(IMUvalue);
                // std::cout << " IMUline num : " << IMUline_num << "th    ";
                
                double IMUtimestamp = std::stod(IMUvalues[0]);
                std::cout.precision(15);
                // std::cout << " IMU timestamp : " << IMUtimestamp << std::endl;

                std::vector<float> Gyro;
                Gyro.resize(3);
                Gyro[0] = std::stof(IMUvalues[1]);
                Gyro[1] = std::stof(IMUvalues[2]);
                Gyro[2] = std::stof(IMUvalues[3]);
                
                Eigen::Matrix4f Rotation = gyroToRotation(Gyro);
                
                if(LastScanTimestamp > IMUtimestamp){
                    IMURotation_integral = Rotation * IMURotation_integral;
                }
                else{
                    Eigen::Matrix4f RT_ = RigToIMU * IMURotation_integral * RigToIMU.inverse();
                    Eigen::Matrix4f RT = RigToLidar.inverse() * RT_ * RigToLidar;
                    LidarRotation = RT;
                    IMURotation_integral = Rotation;
                    IMUline_num++;
                    break;
                }
                
                // std::cout << std::endl;
                IMUline_num++;
            }    

            pcl::PointCloud<pcl::PointXYZ> KeyframeLidarPoints;
            pcl::PointCloud<pcl::PointXYZ> VisualizeNoDistortionPoints;
            for (int j = 0; j < num_seqs; j++){

                

                pcl::PointCloud<pcl::PointXYZ> points;
                // points.clear();
                // points.reserve(kMaxNumberOfPoints);
                
                pcl::PointXYZ point;
                // pcl::PointXYZ point;
                
                
                int& num_pts = lidar_data.num_points;
                ifs.read((char*)&num_pts, sizeof(int));
                
                lidar_data.binary_data.resize((4 * num_pts) * sizeof(float) + 2 * num_pts);
                

                ifs.read((char*) lidar_data.points_ptr(), num_pts * sizeof(float) * 3);
                ifs.read((char*) lidar_data.intensities_ptr(), num_pts );
                
                ifs.read((char*) lidar_data.azimuth_idxs_ptr(), num_pts);    
                ifs.read((char*) &lidar_data.num_blocks, sizeof(int));
                CHECK_LE(lidar_data.num_blocks, num_pts);  
                ifs.read((char*) lidar_data.azimuth_degs_ptr(),
                        lidar_data.num_blocks * sizeof(float));
                ifs.read((char*) &lidar_data.num_channels, sizeof(uint8_t));
                ifs.read((char*) &lidar_data.timestamp_ns, sizeof(int64_t));


                // save 3D points and intensity 
                for(int k = 0; k < num_pts * 3; k+=3){
                    point.x = *(lidar_data.points_ptr() + k);
                    point.y = *(lidar_data.points_ptr() + k + 1);
                    point.z = *(lidar_data.points_ptr() + k + 2);
                    // point.intensity = (((float)*( lidar_data.intensities_ptr() + (k/3) ) ) / 255); // 0 ~ 1 , raw data : 0 ~ 254
                    points.push_back(point);
                }
                
                // std::cout << " before moving " << points[0] << std::endl;
                // MoveDistortionPoints(points, LidarRotation, j, num_seqs);
                // std::cout << " after moving " << points[0] << std::endl;
                
                for(size_t i = 0; i < points.size(); i++){
                    pcl::PointXYZ NoDistortionPoint;
                    NoDistortionPoint.x = points[i].x;
                    NoDistortionPoint.y = points[i].y;
                    NoDistortionPoint.z = points[i].z;
                    
                    pcl::PointXYZ KeyframeLidarPoint;
                    KeyframeLidarPoint.x = points[i].x;
                    KeyframeLidarPoint.y = points[i].y;
                    KeyframeLidarPoint.z = points[i].z;

                    KeyframeLidarPoints.push_back(KeyframeLidarPoint);
                    VisualizeNoDistortionPoints.push_back(NoDistortionPoint);
                }
            }
                
            // timestamp
            ros::Time timestamp_ros;
            timestamp_ros.fromNSec(lidar_data.timestamp_ns);
            std::cout << "Lidar Scan timestamp  : " << timestamp_ros << std::endl;
            
            // publish
            sensor_msgs::PointCloud2 output;
            pcl::toROSMsg(VisualizeNoDistortionPoints, output);
            output.header.stamp = timestamp_ros;
            output.header.frame_id = "/camera_init";
            LaserCloudNoDistortion.publish(output);
            
            if(KeyframeTimestamp < LastScanTimestamp){
                MovePointsToKeyframe(KeyframeLidarPoints, KeyframePose);
                std::cout << "Visualize Keyframe Pointcloud!" << std::endl;
                for(size_t i = 0; i < KeyframeLidarPoints.size(); i++){
                    pcl::PointXYZ VisualizeKeyframeLidarPoint;
                    VisualizeKeyframeLidarPoint.x = KeyframeLidarPoints[i].x;
                    VisualizeKeyframeLidarPoint.y = KeyframeLidarPoints[i].y;
                    VisualizeKeyframeLidarPoint.z = KeyframeLidarPoints[i].z;     

                    VisualizeKeyframeLidarPoints.push_back(VisualizeKeyframeLidarPoint);
                    Keyframe_timestamp = timestamp_ros;
                }

                break;
            }
            

            // publish
            sensor_msgs::PointCloud2 outputs;
            pcl::toROSMsg(VisualizeKeyframeLidarPoints, outputs);
            outputs.header.stamp = timestamp_ros;
            outputs.header.frame_id = "/camera_init";
            KeyframeLaserCloud.publish(outputs);
            
            

   
                
                





                
                

                



                


            r.sleep();
            ifs.close();
            // std::cout << std::endl;
            Lidarline_num++;
        }
        
        Eigen::Quaternionf q = GetQuaternion(KeyframePose);
        nav_msgs::Odometry KeyframePoseOutput;
		KeyframePoseOutput.header.frame_id = "/camera_init";
		KeyframePoseOutput.header.stamp = Keyframe_timestamp;
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

		KeyframePath.header.stamp = KeyframePoseOutput.header.stamp;
		KeyframePath.header.frame_id = "/camera_init";
		KeyframePath.poses.push_back(Keyframepose);
		pubKeyframePath.publish(KeyframePath);
        
        
        KeyframePoseline_num++;    
    }
    LidarcsvFile.close();
    
    printf("XYZ_binary to rosbag done\n");









    return 0;
}