/*
* Time: 2023/09/19 10:56:06
* Author: ZHANG WEN HAO
* Contact: 821298794@qq.com
* Version: 0.1
* Language: c++
* Description:  多相机内外参联合标定
*/

#include "MuiltiCamCalibration.h"


//实例化整个标定过程
MuiltiCamCalibration::MuiltiCamCalibration(std::string cam1_image_dir, std::string cam2_image_dir, std::string visual_chess_dir, cv::Size pattern_size, double chess_grid_size, std::string out_path): cam1_image_dir_(cam1_image_dir),
cam2_image_dir_(cam2_image_dir), visual_chess_dir_(visual_chess_dir), pattern_size_(pattern_size), chess_grid_size_(chess_grid_size), out_path_(out_path)
{

}

MuiltiCamCalibration::~MuiltiCamCalibration()
{

}

//获取各图像的路径，并且有序取出图像
void MuiltiCamCalibration::ReciveImagePath(std::string image_dir,std::vector<std::string>& images_path)
{
    DIR* dir;
    struct dirent* entry;
    if ((dir = opendir(image_dir.c_str())) != nullptr)
    {
        while ((entry = readdir(dir)) != nullptr)
        {
            std::string file_name = entry->d_name;
            if (file_name.size() >= 4 && file_name.substr(file_name.size() - 4) == ".png")
            {
                std::string file_path = image_dir + "/"+ file_name;
                images_path.push_back(file_path);
            }
        }
        closedir(dir);
    } else {
        perror("Error opening directory");
    }
}



//相机内参标定
void MuiltiCamCalibration::CamInstrinsicCalibrate(std::string image_dir,cv::Mat& camera_matrix, cv::Mat& dist_coeffs, std::vector<std::pair<std::string ,Eigen::Matrix4d>>& T_cam2chesses,std::vector<std::pair<std::string ,std::vector<cv::Point2f>>>& images_points)
{
    std::vector<std::string> calibrate_images_path;
    ReciveImagePath(image_dir,calibrate_images_path);
    std::vector<std::vector<cv::Point3f>> object_points;  // 棋盘格角点
    std::vector<std::vector<cv::Point2f>> image_points;   // 二维图像像素点
    std::vector<std::string> image_base_names;

    // 棋盘格世界坐标系的坐标
    std::vector<cv::Point3f> obj;
    for (int i = 0; i < pattern_size_.height; ++i) {
        for (int j = 0; j < pattern_size_.width; ++j) {
            obj.push_back(cv::Point3f(chess_grid_size_*j, chess_grid_size_*i, 0.0f));
        }
    }

    cv::Mat gray_frame,image;
    for(int i = 0;i<calibrate_images_path.size();i++)
    {
        size_t pos_start = calibrate_images_path[i].find_last_of("/");
        size_t pos_end = calibrate_images_path[i].find_last_of(".");
        size_t num = pos_end - pos_start - 1;
        std::string file_name = calibrate_images_path[i].substr(pos_start+1, num); //文件名
        image =  cv::imread(calibrate_images_path[i]);
        image_size_ = image.size();
        cv::cvtColor(image, gray_frame, cv::COLOR_BGR2GRAY);
        std::vector<cv::Point2f> corners;
        bool ret = cv::findChessboardCorners(gray_frame, pattern_size_, corners,
                                             cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);
        //按顺序进行对应添加
        if(ret)
        {
            //角点亚像素提取
            cv::cornerSubPix(gray_frame, corners, cv::Size(11, 11), cv::Size(-1, -1),
                             cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
            //将对应的图像id的棋盘格以及图像点赋值，记录掩膜
            object_points.push_back(obj);
            image_points.push_back(corners);
            image_base_names.push_back(file_name);
            cv::drawChessboardCorners(image, pattern_size_, corners, ret);
            std::string image_out_path = visual_chess_dir_+ "/"+ file_name +".png";
            cv::imwrite(image_out_path,image);
            images_points.push_back(std::make_pair(file_name,corners));
        }
    }
    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;
    // 执行相机内参标定
    cv::calibrateCamera(object_points, image_points,gray_frame.size(), camera_matrix, dist_coeffs, rvecs, tvecs);
    // 打印内参矩阵和畸变系数
    std::cout << "Camera Matrix:\n" << camera_matrix << "\n\n";
    std::cout << "Distortion Coefficients:\n" << dist_coeffs << "\n";

    //将信息进行组合
    //将旋转向量和平移量转为变换矩阵
    for (int i=0; i < rvecs.size(); i++)
    {
        cv::Mat current_rvec = rvecs[i];
        cv::Mat current_tvec = tvecs[i];
        std::string name = image_base_names[i];
        cv::Mat rotation_matrix;
        cv::Rodrigues(current_rvec, rotation_matrix);
        Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
        for(int row=0;row<3;row++)
        {
            for(int col=0;col<3;col++)
            {
                transformation_matrix(row,col) = rotation_matrix.at<double>(row, col);
            }
        }
        transformation_matrix(0,3)=current_tvec.at<double>(0);
        transformation_matrix(1,3)=current_tvec.at<double>(1);
        transformation_matrix(2,3)=current_tvec.at<double>(2);
        T_cam2chesses.push_back(std::make_pair(name,transformation_matrix));
    }
}

void MuiltiCamCalibration::MuiltiCamExtCalibrate(cv::Mat& camera1_matrix,cv::Mat& cam1_dist_coeffs,cv::Mat& camera2_matrix,cv::Mat& cam2_dist_coeffs,std::vector<std::pair<std::string ,std::vector<cv::Point2f>>>& cam1_images_points,std::vector<std::pair<std::string ,std::vector<cv::Point2f>>>& cam2_images_points,cv::Mat T_cam12cam2)
{
    std::vector<std::vector<cv::Point3f>> obj_points;
    std::vector<std::vector<cv::Point2f>> cam1_img_points, cam2_img_points;
    //相机外参标定
    for(int num=0;num<cam1_images_points.size();num++)
    {
        //生成世界坐标系中的点
        std::vector<cv::Point3f> obj;
        for (int i = 0; i < pattern_size_.height; ++i)
        {
            for (int j = 0; j < pattern_size_.width; ++j)
            {
                obj.push_back(cv::Point3f(chess_grid_size_*j, chess_grid_size_*i, 0.0f));
            }
        }
        //获取个点的对应点
        for(int idx=0;idx<cam2_images_points.size();idx++)
        {
            if(cam2_images_points[idx].first == cam1_images_points[num].first)
            {
                obj_points.push_back(obj);
                cam1_img_points.push_back(cam1_images_points[num].second);
                cam2_img_points.push_back(cam2_images_points[num].second);
            }
        }
    }
    cv::Mat  R,T, F, E;
    double rms_error = cv::stereoCalibrate(obj_points, cam1_img_points, cam2_img_points, camera1_matrix, cam1_dist_coeffs, camera2_matrix, cam2_dist_coeffs,image_size_, R, T, E, F);
    T_cam12cam2 = cv::Mat::eye(4, 4, CV_64F); // 创建一个4x4的单位矩阵
    // 将旋转矩阵和平移向量复制到变换矩阵中
    R.copyTo(T_cam12cam2(cv::Rect(0, 0, 3, 3))); // 复制R到左上角3x3子矩阵
    T.copyTo(T_cam12cam2(cv::Rect(3, 0, 1, 3))); // 复制T到右上角3x1子矩阵
    // 打印校准的重投影误差
    std::cout << "校准的重投影误差 (RMS error): " << rms_error << std::endl;
    std::cout<<"T_cam12cam2:"<<T_cam12cam2<<std::endl;
}



int main(int argc, char **argv)
{
    // 读取YAML文件
    YAML::Node config = YAML::LoadFile("../config/setting_config.yaml");
    std::string out_path = config["outfile_path"].as<std::string>();
    cv::Size pattern_size;
    pattern_size.width = config["chess_width"].as<int>(); //棋盘格的宽
    pattern_size.height = config["chess_height"].as<int>(); //棋盘格的高
    double chess_grid_size = config["chess_grid_size"].as<double>(); //棋盘格格子尺寸
    std::string cam1_image_dir = config["cam1_image_dir"].as<std::string>(); //输入图像的目录
    std::string cam2_image_dir = config["cam2_image_dir"].as<std::string>(); //输入激光点的目录
    std::string visual_chess_dir = config["visual_chess_dir"].as<std::string>(); //可视化标定板角点
    //初始化内参标定类
    MuiltiCamCalibration calibrater(cam1_image_dir, cam2_image_dir, visual_chess_dir, pattern_size, chess_grid_size, out_path);
    cv::Mat camera1_matrix,cam1_dist_coeffs,camera2_matrix,cam2_dist_coeffs;
    std::vector<std::pair<std::string ,Eigen::Matrix4d>> T_cam12chesses,T_cam22chesses;
    //标定出相机两个相机的内参
    std::vector<std::pair<std::string ,std::vector<cv::Point2f>>> cam1_images_points,cam2_images_points;

    calibrater.CamInstrinsicCalibrate(cam1_image_dir,camera1_matrix,cam1_dist_coeffs,T_cam12chesses,cam1_images_points); //相机内参标定
    calibrater.CamInstrinsicCalibrate(cam2_image_dir,camera2_matrix,cam2_dist_coeffs,T_cam22chesses,cam2_images_points); //相机内参标定

    cv::Mat T_cam12cam2;
    calibrater.MuiltiCamExtCalibrate(camera1_matrix,cam1_dist_coeffs,camera2_matrix,cam2_dist_coeffs,cam1_images_points,cam2_images_points,T_cam12cam2);

    cv::FileStorage fs(out_path, cv::FileStorage::WRITE);
    // 将Eigen::Matrix4d写入到YAML文件
    fs << "T_cam12cam2" << T_cam12cam2;
    // 将cv::Mat写入到YAML文件
    fs << "camera1_matrix" << camera1_matrix;
    fs << "cam1_dist_coeffs" << cam1_dist_coeffs;
    fs << "camera2_matrix" << camera2_matrix;
    fs << "cam2_dist_coeffs" << cam2_dist_coeffs;
    // 关闭文件
    fs.release();
    std::cout << "标定数据写入完成，写入至："<< out_path<< std::endl;

}