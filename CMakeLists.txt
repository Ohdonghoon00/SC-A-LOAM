cmake_minimum_required(VERSION 2.8.3)
project(aloam_velodyne)

set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  rosbag
  std_msgs
  image_transport
  cv_bridge
  tf
)
find_package( Boost REQUIRED COMPONENTS 
 

timer )
#find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

find_package(OpenMP REQUIRED)
find_package(GTSAM REQUIRED QUIET)

include_directories(
  include
	${catkin_INCLUDE_DIRS} 
	${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
  DEPENDS EIGEN3 PCL 
  INCLUDE_DIRS include
)


add_executable(ascanRegistration src/scanRegistration.cpp)
target_link_libraries(ascanRegistration ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(alaserOdometry src/laserOdometry.cpp)
target_link_libraries(alaserOdometry ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_executable(alaserMapping src/laserMapping.cpp)
target_link_libraries(alaserMapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_executable(alaserPGO 
  src/laserPosegraphOptimization.cpp
  include/scancontext/Scancontext.cpp
)
target_compile_options(alaserPGO 
  PRIVATE ${OpenMP_CXX_FLAGS}
)
target_link_libraries(alaserPGO 
  ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES}
  ${OpenMP_CXX_FLAGS} ${OpenCV_LIBS}
  gtsam
)
# target_link_libraries( alaserPGO ${Boost_PROGRAM_OPTIONS_LIBRARY} ${Boost_REGEX_LIBRARY} )
add_executable(kittiHelper src/kittiHelper.cpp)
target_link_libraries(kittiHelper ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(PublishLidarData src/PublishLidarData.cpp)
target_link_libraries(PublishLidarData glog ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(VisualizePointCloud src/VisualizePointCloud.cpp)
target_link_libraries(VisualizePointCloud glog ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(VisualizeKeyframePointcloud src/VisualizeKeyframePointcloud.cpp)
target_link_libraries(VisualizeKeyframePointcloud glog ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

# test 
add_executable(PublishLidarData2 src/PublishLidarData2.cpp)
target_link_libraries(PublishLidarData2 glog ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

# Save Data
add_executable(SaveData src/SaveData.cpp)
target_link_libraries(SaveData glog ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

# Visualize Pose
add_executable(VisualizePose src/VisualizePose.cpp)
target_link_libraries(VisualizePose glog ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})