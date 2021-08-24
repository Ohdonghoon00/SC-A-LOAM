# SC-A-LOAM
### install

   
    mkdir -p ~/catkin_SC-A-LOAM_ws/src
    cd ~/catkin_SC-A-LOAM_ws/src
    git clone https://github.com/gisbi-kim/SC-A-LOAM.git
    cd ../
    catkin_make
    source ~/catkin_scaloam_ws/devel/setup.bash
   

- `Publish_lidar_data.launch` 파일내 data_dir 경로 수정
### data 구조

    
    data_dir
    ├─ timestamp 
    	└─ lidar_timestamp.csv 			
    ├─ lidar_binary_file 	
    	├─ 00000.xyz
    	├─ 00001.xyz
    	└─ ...						
    └─ multipleye_lidar_data.bag
   

### run

    
    roslaunch aloam_velodyne aloam_velodyne_VLP_16.launch
    roslaunch aloam_velodyne Publish_lidar_data.launch
   
