#include <ros/ros.h>
#include <iostream>
#include <thread>
#include <fstream>

#include <pcl/point_cloud.h> // for PointCloud
#include <pcl/common/io.h> // for copyPointCloud
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>

#include "pose_cam_estimate/PreProcessing.h"
#include "pose_cam_estimate/getCorner.h"
#include "pose_cam_estimate/icp.h"

#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <std_msgs/Float32MultiArray.h>
using namespace std::chrono_literals;

Eigen::Quaterniond qlidarToCamera;
Eigen::Matrix3d lidarToCamera;

std::vector<float> marker_info;

void marker_pose_transformation_callback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    marker_info.clear();
    for(int i=0; i < msg->data.size(); i++){
        marker_info.push_back(msg->data[i]);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lineSegmentation");
    ros::NodeHandle n;

    // message_filters::Subcriber<>         marker_pose
    // message_filters::Subcriber<>         marker_points
    // message_filters::Subscriber <lidar_camera_calibration::marker_6dof> rt_sub(n, "lidar_camera_calibration_rt", 1);
    // message_filters::Subscriber <>
    // TimeSynchronizer<Image, CameraInfo> sync(image_sub, info_sub, 10);

    ros::Subscriber sub = n.subscribe("marker_pose_transformation", 10, marker_pose_transformation_callback);


    //-----------------------------------------------------------------------------------------------
    // initialize PointClouds
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZI>);

    std::vector<int> filenames;
    filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");

    // load file
    pcl::PointCloud<PointXYZRI>::Ptr cloud (new pcl::PointCloud<PointXYZRI> ()); 
    pcl::PointCloud<PointXYZRI>::Ptr final (new pcl::PointCloud<PointXYZRI> ());
    pcl::PointCloud<pcl::PointXYZI>::Ptr visualize_points(new pcl::PointCloud<pcl::PointXYZI> ());
    
    if(pcl::io::loadPCDFile(argv[filenames[0]], *cloud_temp) < 0) {
        std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl;
        return -1;
    }

    cloud = toPointXYZRI(*cloud_temp);
    *cloud = intensityByRangeDiff(*cloud);

    *cloud = transform(*cloud, 0.0, 0.0 , 0.0, 1.57, -1.57, 0);

    // Rotation matrix to transfor lidar point cloud to camera's frame
    qlidarToCamera =  Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ())
                    * Eigen::AngleAxisd(-1.57, Eigen::Vector3d::UnitY())
                    * Eigen::AngleAxisd(1.57, Eigen::Vector3d::UnitX());
    
    lidarToCamera = qlidarToCamera.matrix(); 

    visualize_points = toPointXYZI(*cloud);
    // visualize_points = toPointXYZI(*transformed_cloud);

    // pcl::visualization::PCLVisualizer viewer;
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> cloud_color_handler(visualize_points, 255, 255, 255);
    // viewer.addPointCloud(visualize_points, cloud_color_handler, "original_cloud");
    // viewer.setBackgroundColor(0.05, 0.05, 0.05, 0);
    // viewer.addCoordinateSystem(0.2);

    // while (!viewer.wasStopped()) {
    //     viewer.spinOnce(100);
    //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // }
    // ------------------------------------------------------------------------------------------------------------------------------

    cv::Mat projectionMatrix = cv::Mat(3, 4, CV_32FC1);
    projectionMatrix.at<float>(0,0) = 601.5337524414062;
    projectionMatrix.at<float>(0,1) = 0.0;
    projectionMatrix.at<float>(0,2) = 318.54693603515625;
    projectionMatrix.at<float>(0,3) = 0.0;
    projectionMatrix.at<float>(1,0) = 0.0;
    projectionMatrix.at<float>(1,1) = 601.4232788085938;
    projectionMatrix.at<float>(1,2) = 244.32102966308594;
    projectionMatrix.at<float>(1,3) = 0.0;
    projectionMatrix.at<float>(2,0) = 0.0;
    projectionMatrix.at<float>(2,1) = 0.0;
    projectionMatrix.at<float>(2,2) = 1.0;
    projectionMatrix.at<float>(2,3) = 0.0;

    ros::Rate loop_rate(5);

    bool a = getCorners(*visualize_points, projectionMatrix);

    while(ros::ok()){
        ros::spinOnce();

        if (marker_info.size() > 0){
            
            if(a == true){
                find_transformation(marker_info, 1, 100, lidarToCamera); 
            }
            
        }
        loop_rate.sleep();
    }
    
    

    




}