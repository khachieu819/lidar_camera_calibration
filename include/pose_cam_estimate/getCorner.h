#ifndef GET_CORNER_H_
#define GET_CORNER_H


#include <ros/ros.h>
#include "pose_cam_estimate/PreProcessing.h"
#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp> 
#include <opencv2/imgproc.hpp> 
#include <ros/package.h>

#include <pcl/common/intersections.h>

int iteration_count = 0;
int num_of_markers = 1;
std::vector<std::vector<cv::Point>> stored_corners;


//------------------------------- Project lidar 3d in camera frame to 2d image ----------------------------
cv::Point project(const pcl::PointXYZI &pt, const cv::Mat & projection_matrix)
{
    cv::Mat pt_3D(4, 1, CV_32FC1);
    pt_3D.at<float>(0) = pt.x;
    pt_3D.at<float>(1) = pt.y;
    pt_3D.at<float>(2) = pt.z;
    pt_3D.at<float>(3) = 1.0f;

    cv::Mat pt_2D = projection_matrix * pt_3D;
    float w = pt_2D.at<float>(2);
    float x = pt_2D.at<float>(0) / w;
    float y = pt_2D.at<float>(1) / w;

    return cv::Point(x, y);
}

cv::Mat project(cv::Mat projection_matrix, cv::Rect frame, pcl::PointCloud <pcl::PointXYZI> point_cloud)
{
    cv::Mat plane = cv::Mat::zeros(frame.size(), CV_32FC1);

    for(pcl::PointCloud<pcl::PointXYZI>::iterator pt = point_cloud.begin(); pt < point_cloud.end(); pt++)
    {
        if(pt->z < 0){
            continue;
        }

        cv::Point xy = project(*pt, projection_matrix);

        if(xy.inside(frame)){
            plane.at<float>(xy) = 250;
        }
    }

    cv::Mat plane_gray;
    cv::normalize(plane, plane_gray, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    cv::dilate(plane_gray, plane_gray, cv::Mat());

    return plane_gray;
}
 



bool getCorners(pcl::PointCloud<pcl::PointXYZI> scan, cv::Mat projection_matrix)
{

    // scan: Point in edege or marker and has been transform to camera coordinate

    ROS_INFO_STREAM("iteration number: " << iteration_count << "\n");

    cv::Mat temp_mat(1080, 1920, CV_8UC3);

    pcl::PointCloud <pcl::PointXYZI> pc = scan;
    cv::Rect frame(0, 0, temp_mat.cols, temp_mat.rows);
    cv::Mat image_edge_laser = project(projection_matrix, frame, scan);     // lidar data in camera coordinate projected to 2D plane
    cv::threshold(image_edge_laser, image_edge_laser, 10, 255, 0);

    cv::Mat polygon_image(image_edge_laser.size(), CV_8UC3, cv::Scalar(0, 0, 0)); 
    cv::Mat combined_rgb_laser;
    std::vector <cv::Mat> rgb_laser_channels;

    rgb_laser_channels.push_back(image_edge_laser);
    rgb_laser_channels.push_back(cv::Mat::zeros(image_edge_laser.size(), CV_8UC1));
    rgb_laser_channels.push_back(temp_mat);

    std::cout << combined_rgb_laser.size() << std::endl;
    cv::merge(rgb_laser_channels, combined_rgb_laser);

    // std::cout << combined_rgb_laser << std::endl;

    std::map <std::pair<int, int>, std::vector<float>> c2D_to_3D;
    std::vector<float> point_3D;

    // store correspondences

    for(pcl::PointCloud<pcl::PointXYZI>::iterator pt = pc.begin(); pt < pc.end(); pt++)
    {
        // ignore points behind the camera
        if (pt->z < 0){
            continue;
        }

        cv::Point xy = project(*pt, projection_matrix);
        if(xy.inside(frame)){
            // create a Map of 2D and 3D points (Lien ket du lieu tren anh 2D voi cac diem 3D point cloud tren camera coordinate)
            point_3D.clear();
            point_3D.push_back(pt->x);
            point_3D.push_back(pt->y);
            point_3D.push_back(pt->z);
            c2D_to_3D[std::pair<int, int>(xy.x, xy.y)] = point_3D;
        }
    }

    // Get region of interest
    // Creating a vector with 4 elements has value is 4
    std::vector<int> LINE_SEGMENTS(num_of_markers, 4);          // assume each has 4 edges and 4 corners
    pcl::PointCloud<pcl::PointXYZI>::Ptr board_corners(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr marker(new pcl::PointCloud<pcl::PointXYZI>);
    
    std::vector<cv::Point3f> C_3D;      // corner 3D
    std::vector<cv::Point3f> C_2D;      // corner 2D

    std::string pkg_loc = ros::package::getPath("pose_cam_estimate");
    std::ofstream outfile(pkg_loc + "/config/points.txt", std::ios_base::trunc);
    outfile << num_of_markers * 4 << "\n";


    for(int q=0; q < num_of_markers; q++){
        std::cout << "-------Moving to next marker --------------" << std::endl;
        std::vector<Eigen::VectorXf> line_model;

        for(int i=0; i < LINE_SEGMENTS[q]; i++){
            cv::Point _point_;
            std::vector<cv::Point> polygon;
            int collected;

            if(iteration_count == 0){
                polygon.clear();
                collected = 0;
                while(collected != LINE_SEGMENTS[q])
                {
                    cv::setMouseCallback("cloud", onMouse, &_point_);
                    cv::imshow("cloud", image_edge_laser);
                    cv::waitKey(0);
                    if(_point_.x != 0 && _point_.y !=0){
                        polygon.push_back(_point_);
                        ++collected;
                    }
                    
                }
                stored_corners.push_back(polygon);
            }

            polygon = stored_corners[4 * q + i];

            std::cout << polygon.size() << std::endl;

            rgb_laser_channels.clear();
            rgb_laser_channels.push_back(image_edge_laser);
            rgb_laser_channels.push_back(cv::Mat::zeros(image_edge_laser.size(), CV_8UC1));
            rgb_laser_channels.push_back(cv::Mat::zeros(image_edge_laser.size(), CV_8UC1));
            cv::merge(rgb_laser_channels, combined_rgb_laser);


            
            for(int j=0; j < 4; j++){ 
                cv::line(combined_rgb_laser, polygon[j], polygon[(j+1)%4], cv::Scalar(0, 255,0), 2);
            }

            // initialize PointClouds
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud <pcl::PointXYZI>);
            pcl::PointCloud<pcl::PointXYZI>::Ptr final(new pcl::PointCloud <pcl::PointXYZI>);

            for(std::map<std::pair<int, int>, std::vector<float>>::iterator 
                    it = c2D_to_3D.begin(); it != c2D_to_3D.end(); ++it)
            {
                if(cv::pointPolygonTest(cv::Mat(polygon), cv::Point(it->first.first, it->first.second), true) > 0)
                {
                    cloud->push_back(pcl::PointXYZI(it->second[0], it->second[1], it->second[2]));       // std::vector<float> ; size = 3
                    rectangle(combined_rgb_laser,
                            cv::Point(it->first.first, it->first.second),
                            cv::Point(it->first.first, it->first.second),
                            cv::Scalar(0, 0, 255), 3, 8,
                            0);
                }
            }

            if (cloud->size() < 2){
                return false;
            }

            cv::imshow("polygon", combined_rgb_laser);
            cv::waitKey(4);

            std::vector<int> inliers;
            Eigen::VectorXf model_coefficients;

            // Create RandomSampleConsensus object and comput the appropriated model
            pcl::SampleConsensusModelLine<pcl::PointXYZI>::Ptr model_l(new pcl::SampleConsensusModelLine<pcl::PointXYZI>(cloud));
            pcl::RandomSampleConsensus<pcl::PointXYZI> ransac(model_l);
            ransac.setDistanceThreshold(0.01);
            ransac.computeModel();
            ransac.getInliers(inliers);
            ransac.getModelCoefficients(model_coefficients);
            line_model.push_back(model_coefficients);

            // std::cout << "Line coefficients are:" << "\n" << model_coefficients << "\n";

            // copy all inliers of the model computed to another PointCloud
            pcl::copyPointCloud<pcl::PointXYZI> (*cloud, inliers, *final);
            *marker += *final;
        }

        // calculate approximate intersection of lines
        Eigen::Vector4f p1, p2, p_intersect;
        pcl::PointCloud<pcl::PointXYZI>::Ptr corners(new pcl::PointCloud<pcl::PointXYZI>);
        for(int i=0; i < LINE_SEGMENTS[q]; i++)
        {
            
            pcl::lineToLineSegment(line_model[i], line_model[(i+1) % LINE_SEGMENTS[q]], p1, p2);
            for(int j=0; j < 4; j++)
            {
                p_intersect(j) = (p1(j) + p2(j)) / 2.0;
            }
            C_3D.push_back(cv::Point3f(p_intersect(0), p_intersect(1), p_intersect(2)));
            corners->push_back(pcl::PointXYZI(p_intersect(0), p_intersect(1), p_intersect(2)));
            // std::cout << "Point of intersection is approximately: \n" << p_intersect << "\n";
            // std::cout << p_intersect(0) << " " << p_intersect(1) << " " << p_intersect(2) << "\n";
            outfile << p_intersect(0) << " " << p_intersect(1) << " " << p_intersect(2) << "\n";
        }

        *board_corners += *corners;
    }

    for(pcl::PointCloud<pcl::PointXYZI>::iterator corner_point = board_corners->begin(); corner_point < board_corners->end(); corner_point++){
        std::cout << *corner_point << std::endl;
    }

    iteration_count++;
    return true;
}




















#endif
