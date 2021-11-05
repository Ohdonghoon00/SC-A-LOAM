


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

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <glog/logging.h>





ros::Publisher LaserCloud;

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

    // publish delay
    ros::Rate r(10.0 / publish_delay);
    ros::Rate r_(1);
    


    
    // Lidar timestamp.csv path
    std::string LidarcsvPath = data_dir + "lidar_timestamp.csv";
    std::ifstream LidarcsvFile(LidarcsvPath, std::ifstream::in);

    if(!LidarcsvFile.is_open())
    {
        std::cout << " Lidar csv file failed to open " << std::endl;
        return EXIT_FAILURE;
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
        for (int j = 0; j < num_seqs; j++)
        {

            

            pcl::PointCloud<pcl::PointXYZI> points;
            // pcl::PointCloud<pcl::PointXYZ> points;
            points.clear();
            points.reserve(kMaxNumberOfPoints);
            
            pcl::PointXYZI point;
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
                points.push_back(point);
            }
            
            std::cout << " points num :  " << points.size() << "    ";

            // timestamp
            ros::Time timestamp_ros;
            timestamp_ros.fromNSec(lidar_data.timestamp_ns);
            std::cout << timestamp_ros << std::endl;

            // publish
            sensor_msgs::PointCloud2 output;
            pcl::toROSMsg(points, output);
            output.header.stamp = timestamp_ros;
            output.header.frame_id = "/camera_init";
            LaserCloud.publish(output);
        

            r_.sleep();
        }



            



        r.sleep();
        ifs.close();
        std::cout << std::endl;
        line_num++;    
    }

    LidarcsvFile.close();
    
    printf("XYZ_binary to rosbag done\n");









    return 0;
}