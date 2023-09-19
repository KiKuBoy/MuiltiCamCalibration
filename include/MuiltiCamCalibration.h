/*
* Time: 2023/09/19 10:56:06
* Author: ZHANG WEN HAO
* Contact: 821298794@qq.com
* Version: 0.1
* Language: c++
* Description:  多相机内外参联合标定
*/

#ifndef LIDARCAMCALIBRATION_2DLIDARCAMCALIBRATION_H
#define LIDARCAMCALIBRATION_2DLIDARCAMCALIBRATION_H
#include <iostream>
#include <fstream>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <dirent.h>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include "yaml-cpp/yaml.h"


struct Oberserve
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Oberserve()
    {
        chess_pose_qca = Eigen::Quaterniond(1,0,0,0);
        chess_pose_tca = Eigen::Vector3d::Zero();
    }

    Eigen::Quaterniond chess_pose_qca;    // 该时刻chessboard在相机中的姿态
    Eigen::Vector3d chess_pose_tca;       // 该时刻chessboard在相机中的偏移
    std::vector<Eigen::Vector3d> points;  // 该时刻打在标定板上的激光点云数据
};



class MuiltiCamCalibration {
public:
    MuiltiCamCalibration(std::string cam1_image_dir, std::string cam2_image_dir, std::string visual_chess_dir, cv::Size pattern_size, double chess_grid_size, std::string out_path);
    ~MuiltiCamCalibration();
    void CamInstrinsicCalibrate(std::string image_dir,cv::Mat& camera_matrix,cv::Mat& dist_coeffs,std::vector<std::pair<std::string,Eigen::Matrix4d>>& T_cam2chesses,std::vector<std::pair<std::string ,std::vector<cv::Point2f>>>& images_points);
    void MuiltiCamExtCalibrate(cv::Mat& camera1_matrix,cv::Mat& cam1_dist_coeffs,cv::Mat& camera2_matrix,cv::Mat& cam2_dist_coeffs,std::vector<std::pair<std::string ,std::vector<cv::Point2f>>>& cam1_images_points,std::vector<std::pair<std::string ,std::vector<cv::Point2f>>>& cam2_images_points,cv::Mat T_cam12cam2);
private:
    void ReciveImagePath(std::string image_dir,std::vector<std::string>& images_path);

public:
    std::string cam1_image_dir_,cam2_image_dir_,visual_chess_dir_;
    cv::Size pattern_size_,image_size_;
    double chess_grid_size_;
    std::string out_path_;
private:

};


#endif //LIDARCAMCALIBRATION_2DLIDARCAMCALIBRATION_H
