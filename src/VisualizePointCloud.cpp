


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





ros::Publisher LaserCloud;
ros::Publisher LaserCloudNoDistortion;

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

void MoveDistortionPoints(pcl::PointCloud<pcl::PointXYZ> &points, std::vector<Eigen::Matrix4f> LidarRotation, int ScanStepNum, int LineNum)
{
    int PointNum = points.size();
    Eigen::MatrixXf MatrixPoints(4, PointNum);
    for(int i = 0; i < PointNum; i++){
        MatrixPoints(0, i) = points[i].x;
        MatrixPoints(1, i) = points[i].y;
        MatrixPoints(2, i) = points[i].z;
        MatrixPoints(3, i) = 1.0;
    }
    
    
    
    int index = ScanStepNum / 4; 
    
    Eigen::Matrix4Xf MatrixPoints_ = LidarRotation[index + 20 * LineNum] * MatrixPoints;
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

    ros::init(argc, argv, "Visualize_PointCloud");
    ros::NodeHandle nh("~");

    LidarData lidar_data;
    
    
    std::string data_dir;
    int publish_delay;
    nh.getParam("data_dir", data_dir);
    nh.getParam("publish_delay", publish_delay);
    
    LaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/aft_pgo_map", 100);
    LaserCloudNoDistortion = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_map", 100);

    // publish delay
    ros::Rate r(10.0 / publish_delay);
    ros::Rate r_(1);
    const Eigen::Matrix4f RigToIMU = To44RT(imu2rig_pose);
    const Eigen::Matrix4f RigToLidar = To44RT(lidar2rig_pose);


    
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

    std::string IMUcsvline;
    int IMUline_num = 0;
    std::vector<Eigen::Matrix4f> RT_LidarPoint;
    
    while(std::getline(IMUcsvFile, IMUcsvline) && ros::ok())
    {
        if(IMUline_num == 0) 
        {
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

        std::stringstream ss(IMUcsvline);
        while(std::getline(ss, IMUvalue, ','))
            IMUvalues.push_back(IMUvalue);
        std::cout << IMUline_num << "th    ";

        std::cout << std::endl;

        std::vector<float> Gyro;
        Gyro.resize(3);
        Gyro[0] = std::stof(IMUvalues[1]);
        Gyro[1] = std::stof(IMUvalues[2]);
        Gyro[2] = std::stof(IMUvalues[3]);
        
        Eigen::Matrix4f Rotation = gyroToRotation(Gyro);
        // std::cout << Rotation << std::endl;
        Eigen::Matrix4f RT_ = RigToIMU * Rotation * RigToIMU.inverse();
        Eigen::Matrix4f RT = RigToLidar.inverse() * RT_ * RigToLidar;
        RT_LidarPoint.push_back(RT);
        
        std::cout << std::endl;
        IMUline_num++;
    }    


    std::string Lidarcsvline;
    int line_num = 0;
    
    // read timestamp.csv
    while(std::getline(LidarcsvFile, Lidarcsvline) && ros::ok())
    {
        if(line_num == 0) 
        {
            line_num++;
            continue;
        }
        std::string value;
        std::vector<std::string> values;
        
        // values[0] -> First Seq Timestamp (ns)
        // values[1] -> Last Seq Timestamp (ns)
        // values[2] -> Fidx
        // values[3] -> Num pts
        // values[4] -> Date
        
        std::stringstream ss(Lidarcsvline);
        while(std::getline(ss, value, ','))
            values.push_back(value);
        int fidx = std::stoi(values[2]);
        
        // binary data path
        std::stringstream Lidar_binary_path;
        Lidar_binary_path << data_dir + "lidar/" << std::setfill('0') << std::setw(5) << fidx << ".xyz";
        
        std::ifstream ifs(Lidar_binary_path.str(), std::ifstream::in);
        
        if (!ifs.is_open()) 
        {
            std::cout << "xyz file failed to open: " << std::endl;
            return EXIT_FAILURE;
        }        

        
        const size_t kMaxNumberOfPoints = 1e6; 
        
        
        std::cout << " File number : " << fidx << "     " << std::endl;
        
        // read binary data file
        int num_seqs = 0;
        ifs.read((char*)&num_seqs, sizeof(int));
        std::cout << " num_seqs : " << num_seqs << std::endl;
        pcl::PointCloud<pcl::PointXYZ> VisualizePoints;
        pcl::PointCloud<pcl::PointXYZ> VisualizeNoDistortionPoints;
        for (int j = 0; j < num_seqs; j++)
        {

            

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

            // IMU data publish

            std::cout << lidar_data.timestamp_ns << std::endl;
            // save 3D points and intensity 
            for(int k = 0; k < num_pts * 3; k+=3)
            {
                point.x = *(lidar_data.points_ptr() + k);
                point.y = *(lidar_data.points_ptr() + k + 1);
                point.z = *(lidar_data.points_ptr() + k + 2);
                // point.intensity = (((float)*( lidar_data.intensities_ptr() + (k/3) ) ) / 255); // 0 ~ 1 , raw data : 0 ~ 254
                VisualizePoints.push_back(point);
                points.push_back(point);
            }
            
            MoveDistortionPoints(points, RT_LidarPoint, j, line_num);

            for(int i = 0; i < points.size(); i++){
                pcl::PointXYZ NoDistortionPoint;
                NoDistortionPoint.x = points[i].x;
                NoDistortionPoint.y = points[i].y;
                NoDistortionPoint.z = points[i].z;
                
                VisualizeNoDistortionPoints.push_back(NoDistortionPoint);
            }
        }
            // timestamp
            ros::Time timestamp_ros;
            timestamp_ros.fromNSec(lidar_data.timestamp_ns);
            std::cout << timestamp_ros << std::endl;

            // publish
            sensor_msgs::PointCloud2 output;
            pcl::toROSMsg(VisualizePoints, output);
            output.header.stamp = timestamp_ros;
            output.header.frame_id = "/camera_init";
            LaserCloud.publish(output);

            
            
            sensor_msgs::PointCloud2 outputs;
            pcl::toROSMsg(VisualizeNoDistortionPoints, outputs);
            outputs.header.stamp = timestamp_ros;
            outputs.header.frame_id = "/camera_init";
            LaserCloudNoDistortion.publish(outputs);

            



            



            r.sleep();
            ifs.close();
            std::cout << std::endl;
            line_num++;    
    }

    LidarcsvFile.close();
    
    printf("XYZ_binary to rosbag done\n");









    return 0;
}