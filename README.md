# SC-A-LOAM
### Install

   
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/src
    git clone https://github.com/Ohdonghoon00/SC-A-LOAM.git
    cd ../
    catkin_make
    source ~/catkin_ws/devel/setup.bash
   

- `Publish_lidar_data.launch` 파일내 data_dir 경로 수정
### Data Structure

    
    data_dir
    ├─ lidar_timestamp.csv 			
    ├─ lidar	
    	├─ 00000.xyz
    	├─ 00001.xyz
    	└─ ...						
    ├─ imu_data.csv
    └─ capture.yaml
   

### Run
 - SC-A-LOAM 실행
 - `Publish_lidar_data.launch` UndistortionPoints parameter -> true : Move Distortion points // false : Use ordinary points 
   
    `roslaunch aloam_velodyne aloam_velodyne_VLP_16.launch
    roslaunch aloam_velodyne Publish_lidar_data.launch`
 
 - Lidar scan Visualize 실행 ( distortion scan points 와 undistortion scan points)
    
    `roslaunch aloam_velodyne Visualize_PointCloud.launch`
 
 ### Reference
 https://github.com/gisbi-kim/SC-A-LOAM
