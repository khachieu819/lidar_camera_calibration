#ifndef ICP_H_
#define ICP_H_

#include <iostream>
#include <string>
#include <ros/ros.h>
#include <ros/package.h>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>

using namespace Eigen;

Eigen::Vector3d translation_sum;
Eigen::Quaterniond rotation_sum;
Eigen::Matrix3d rotation_avg_by_mult;

float rmse_avg;

int iteration_counter = 0;

std::string pkg_loc = ros::package::getPath("pose_cam_estimate");

Eigen::Quaterniond addQ(Eigen::Quaterniond a, Eigen::Quaterniond b) {
    Eigen::Quaterniond retval;
    if (a.x() * b.x() + a.y() * b.y() + a.z() * b.z() + a.w() * b.w() < 0.0) {
        b.x() = -b.x();
        b.y() = -b.y();
        b.z() = -b.z();
        b.w() = -b.w();
    }
    retval.x() = a.x() + b.x();
    retval.y() = a.y() + b.y();
    retval.z() = a.z() + b.z();
    retval.w() = a.w() + b.w();
    return retval;
}

std::pair<MatrixXd, MatrixXd> readArray() {

    int num_points = 4;
    MatrixXd lidar(3, num_points), camera(3, num_points);

    std::ifstream infile(pkg_loc + "/config/points.txt");

    if (!infile) { // Check if the file opened successfully
        std::cerr << "Error opening file." << std::endl;
    }
    else{
        infile >> num_points;
        for (int i = 0; i < num_points; i++) {
            infile >> lidar(0, i) >> lidar(1, i) >> lidar(2, i);
        }
    }
    infile.close();
    

    std::ifstream infile2(pkg_loc + "/config/points_camera.txt");
    
    for (int i = 0; i < num_points; i++) {
        infile2 >> camera(0, i) >> camera(1, i) >> camera(2, i);
    }

    infile2.close();

    // std::cout << lidar << camera << std::endl;
    return std::pair<MatrixXd, MatrixXd> (lidar, camera);
}

// calculates rotation and translation that transforms points in the lidar frame to the camera frame
Matrix4d calc_RT(MatrixXd lidar, MatrixXd camera, int MAX_ITERS, Eigen::Matrix3d lidarToCamera) {
    if (iteration_counter == 0) {
        // std::ofstream clean_file(pkg_loc + "/log/avg_values.txt", std::ios_base::trunc);
        // clean_file.close();

        translation_sum << 0.0, 0.0, 0.0;
        rotation_sum = Quaterniond(0.0, 0.0, 0.0, 0.0);
        rotation_avg_by_mult << 1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0;
        rmse_avg = 0.0;
    }
    int num_points = lidar.cols();
    std::cout << "Number of points: " << num_points << std::endl;
    Vector3d mu_lidar, mu_camera;

    mu_lidar << 0.0, 0.0, 0.0;
    mu_camera << 0.0, 0.0, 0.0;

    for (int i = 0; i < num_points; i++) {
        mu_lidar(0) += lidar(0, i);
        mu_lidar(1) += lidar(1, i);
        mu_lidar(2) += lidar(2, i);
    }
    for (int i = 0; i < num_points; i++) {
        mu_camera(0) += camera(0, i);
        mu_camera(1) += camera(1, i);
        mu_camera(2) += camera(2, i);
    }

    mu_lidar = mu_lidar / num_points;
    mu_camera = mu_camera / num_points;

    if (iteration_counter == 0) {
        std::cout << "mu_lidar: \n" << mu_lidar << std::endl;
        std::cout << "mu_camera: \n" << mu_camera << std::endl;
    }

    MatrixXd lidar_centered = lidar.colwise() - mu_lidar;
    MatrixXd camera_centered = camera.colwise() - mu_camera;

    if (iteration_counter == 0) {
        std::cout << "lidar_centered: \n" << lidar_centered << std::endl;
        std::cout << "camera_centered: \n" << camera_centered << std::endl;
    }

    Matrix3d cov = camera_centered * lidar_centered.transpose();

    std::cout << cov << std::endl;

    JacobiSVD <MatrixXd> svd(cov, ComputeFullU | ComputeFullV);

    Matrix3d rotation;
    rotation = svd.matrixU() * svd.matrixV().transpose();
    if (rotation.determinant() < 0) {
        Vector3d diag_correct;
        diag_correct << 1.0, 1.0, -1.0;

        rotation = svd.matrixU() * diag_correct.asDiagonal() * svd.matrixV().transpose();
    }

    Vector3d translation = mu_camera - rotation * mu_lidar;

    // averaging translation and rotation
    translation_sum += translation;
    Quaterniond temp_q(rotation);
    rotation_sum = addQ(rotation_sum, temp_q);

    // averaging rotations by multiplication
    rotation_avg_by_mult = rotation_avg_by_mult.pow(1.0 * iteration_counter / (iteration_counter + 1)) *
                           rotation.pow(1.0 / (iteration_counter + 1));

    Vector3d ea = rotation.eulerAngles(2, 1, 0);

    std::cout << "Rotation matrix: \n" << rotation << std::endl;
    std::cout << "Rotation in Euler angles: \n" << ea * 57.3 << std::endl;
    std::cout << "Translation: \n" << translation << std::endl;

    MatrixXd eltwise_error = (camera - ((rotation * lidar).colwise() + translation)).array().square().colwise().sum();
    double error = sqrt(eltwise_error.sum() / num_points);
    std::cout << "RMSE: " << error << std::endl;

    rmse_avg = rmse_avg + error;

    Matrix4d T;
    T.setIdentity(4, 4);
    T.topLeftCorner(3, 3) = rotation;
    T.col(3).head(3) = translation;

    std::cout << "Rigid-body transformation: \n" << T << std::endl;

    iteration_counter++;

    if (iteration_counter % 1 == 0) {
        std::ofstream log_avg_values(pkg_loc + "/log/avg_values.txt", std::ios_base::app);

        std::cout << "--------------------------------------------------------------------\n";
        std::cout << "After " << iteration_counter << " iterations\n";
        std::cout << "--------------------------------------------------------------------\n";

        std::cout << "Average translation is:" << "\n" << translation_sum / iteration_counter << "\n";
        log_avg_values << iteration_counter << "\n";
        log_avg_values << translation_sum / iteration_counter << "\n";


        rotation_sum.x() = rotation_sum.x() / iteration_counter;
        rotation_sum.y() = rotation_sum.y() / iteration_counter;
        rotation_sum.z() = rotation_sum.z() / iteration_counter;
        rotation_sum.w() = rotation_sum.w() / iteration_counter;
        double mag = sqrt(rotation_sum.x() * rotation_sum.x() +
                          rotation_sum.y() * rotation_sum.y() +
                          rotation_sum.z() * rotation_sum.z() +
                          rotation_sum.w() * rotation_sum.w());
        rotation_sum.x() = rotation_sum.x() / mag;
        rotation_sum.y() = rotation_sum.y() / mag;
        rotation_sum.z() = rotation_sum.z() / mag;
        rotation_sum.w() = rotation_sum.w() / mag;

        Eigen::Matrix3d rotation_avg = rotation_sum.toRotationMatrix();
        std::cout << "Average rotation is:" << "\n" << rotation_avg << "\n";
        std::cout << "Average rotation is:" << "\n" << lidarToCamera << "\n";
        Eigen::Matrix3d final_rotation = rotation_avg * lidarToCamera ;
        Eigen::Vector3d final_angles = final_rotation.eulerAngles(2, 1, 0);

        log_avg_values << std::fixed << std::setprecision(8)
                       << rotation_avg(0, 0) << " " << rotation_avg(0, 1) << " " << rotation_avg(0, 2) << "\n"
                       << rotation_avg(1, 0) << " " << rotation_avg(1, 1) << " " << rotation_avg(1, 2) << "\n"
                       << rotation_avg(2, 0) << " " << rotation_avg(2, 1) << " " << rotation_avg(2, 2) << "\n";

        Matrix4d T;
        T.setIdentity(4, 4);
        T.topLeftCorner(3, 3) = rotation_avg;
        T.col(3).head(3) = translation_sum / iteration_counter;
        std::cout << "Average transformation is: \n" << T << "\n";
        std::cout << "Final rotation is:" << "\n" << final_rotation << "\n";
        std::cout << "Final ypr is:" << "\n" << final_angles << "\n";

        std::cout << "Average RMSE is: " << rmse_avg * 1.0 / iteration_counter << "\n";

        MatrixXd eltwise_error_temp = (camera - ((rotation_avg * lidar).colwise() + (translation_sum /
                                                                                     iteration_counter))).array().square().colwise().sum();
        double error_temp = sqrt(eltwise_error_temp.sum() / num_points);

        std::cout << "RMSE on average transformation is: " << error_temp << std::endl;
        log_avg_values << std::fixed << std::setprecision(8) << error_temp << "\n";

    }

    return T;
}

void readArucoPose(std::vector<float> marker_info, int num_of_marker_in_config){
    std::vector<Matrix4d> marker_pose;

    std::cout << marker_info.size() << std::endl;
    ROS_ASSERT(marker_info.size() / 6 == num_of_marker_in_config);

    int j = 0;
    for(int i = 0; i < marker_info.size() / 6; i++)
    {
        Vector3d trans, rot;
        int marker_id = marker_info[j++];
        trans(0) = marker_info[j++];
        trans(1) = marker_info[j++];
        trans(2) = marker_info[j++];
        rot(0) = marker_info[j++];
        rot(1) = marker_info[j++];
        rot(2) = marker_info[j++];

        Transform<double, 3, Affine> aa;
        aa = AngleAxis<double> (rot.norm(), rot / rot.norm());

        Matrix4d g;
        g.setIdentity(4, 4);
        g = aa * g;

        Matrix4d T;
        T.setIdentity(4, 4);
        T.topLeftCorner(3, 3) = g.topLeftCorner(3, 3);  // transpose()
        T.col(3).head(3) = trans;

        marker_pose.push_back(T);
    }

    

    // std::ifstream infile(pkg_loc + "/config/marker_coordinate.txt");
    std::ifstream infile;
    infile.open(pkg_loc + "/config/marker_coordinate.txt");
    if (infile.is_open()) {

        int num_of_markers;
        infile >> num_of_markers;
        MatrixXd points_board;
        points_board.resize(4, 4);

        for(int i=0; i < num_of_markers; i++){
        float temp;
        std::vector<float> board;
        for(int j=0; j < 5; j++){
            infile >> temp;
            board.push_back(temp / 100.0);
        }

        float la, ba;
        la = board[4] / 2 + board[2];
        ba = board[4] / 2 + board[3];

        points_board << ba, 0, board[0] - la, 1,
                    ba - board[1], 0, board[0] - la, 1,
                    ba - board[1], 0, -la, 1,
                    ba, 0, -la, 1;

        // std::cout << points_board.transpose() << std::endl;
        // std::cout << marker_pose[i] << std::endl;
        
        Eigen::MatrixXd C = marker_pose[i] * points_board.transpose(); // Result will be 2x2
        // std::cout << "Result of A * B:\n" << C << std::endl;
        

        std::ofstream outfile(pkg_loc + "/config/points_camera.txt", std::ios_base::trunc);
        if (outfile.is_open()) {
            for(int k=0; k <= 3; k++){
                outfile << C(0, k) << " " << C(2, k) << " " << C(1, k) << " " << "\n";
            }

            outfile.close();
        } else {
            std::cout << "error" << std::endl;
            // Handle error: file could not be opened
        }
        }

        
    }
    infile.close();


    
    

    
    
}

void find_transformation(std::vector<float> marker_info, int num_of_marker_in_config, int MAX_ITERS, Eigen::Matrix3d lidarToCamera)
{
    readArucoPose(marker_info, num_of_marker_in_config);
    std::pair<MatrixXd, MatrixXd> point_clouds = readArray();
    Matrix4d T = calc_RT(point_clouds.first, point_clouds.second, 100, lidarToCamera);
}

#endif