#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <iostream>
#include <string>
#include <vector>

cv::Size patternsize_(11,8);
class ImageSaver
{
public:
    ImageSaver() : nh_("~"), save_images_(false),image_idx_(0)
    {
        image1_sub_.subscribe(nh_, "/camera_01/rgb/image_raw", 1);
        image2_sub_.subscribe(nh_, "/camera_02/rgb/image_raw", 1);

        sync_.reset(new Sync(MySyncPolicy(10), image1_sub_, image2_sub_));
        sync_->registerCallback(boost::bind(&ImageSaver::imageCallback, this, _1, _2));

        save_dir_ = "/home/iimt/image";  // 设置保存图像的目录
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& image1_msg, const sensor_msgs::ImageConstPtr& image2_msg)
    {
            cv_bridge::CvImagePtr cv_image1, cv_image2;
            try
            {
                cv_image1 = cv_bridge::toCvCopy(image1_msg, sensor_msgs::image_encodings::BGR8);
                cv_image2 = cv_bridge::toCvCopy(image2_msg, sensor_msgs::image_encodings::BGR8);
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }
            
            cv::Mat src_mat_1,src_mat_2,rgb_1,rgb_2;
            rgb_1 = cv_image1->image;
            rgb_2 = cv_image2->image;
            rgb_1.copyTo(src_mat_1);
            rgb_2.copyTo(src_mat_2);
	

            char key = cv::waitKey(10);
            
            if (key == 'q')
            {
                std::vector<cv::Point2f> corners_1,corners_2;
                cv::Mat gray_image_1,gray_image_2;
                cvtColor(cv_image1->image,gray_image_1,cv::COLOR_BGR2GRAY);
                cvtColor(cv_image2->image,gray_image_2,cv::COLOR_BGR2GRAY);
                
                
                bool ret_1 = cv::findChessboardCorners(gray_image_1,patternsize_,corners_1,cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);
                bool ret_2 = cv::findChessboardCorners(gray_image_2,patternsize_,corners_2,cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);
                
                if(ret_1 && ret_2)
                {
                    cv::drawChessboardCorners(rgb_1,patternsize_,cv::Mat(corners_1),ret_1);
                    bitwise_not(rgb_1,rgb_1);
                    
                    cv::drawChessboardCorners(rgb_2,patternsize_,cv::Mat(corners_2),ret_2);
                    bitwise_not(rgb_2,rgb_2);
                    // 保存图像
                    saveImage(src_mat_1, "camera_1");
                    saveImage(src_mat_2, "camera_2");
                    ROS_INFO("Saving  images...");
                    image_idx_++;
                }
                else
                {
                    ROS_INFO("Can not get chessboard corners...");
                }
                

            }
            else if (key == 27)  // 'Esc' key
            {
               ros::shutdown(); 
            }
            std::cout<<"there are "<< image_idx_ <<" images!"<<std::endl;
            
            cv::imshow("Image 1", rgb_1);
            cv::imshow("Image 2", rgb_2);
	     
    }

    void saveImage(const cv::Mat& image, const std::string& camera_name)
    {
        // 生成文件名，格式为 camera_name_时间戳.png
        std::string filename = camera_name + "/" + std::to_string(image_idx_) + ".png";
        std::string file_path = save_dir_ + "/" + filename;

        // 保存图像
        cv::imwrite(file_path, image);
        ROS_INFO("Image saved: %s", file_path.c_str());
    }

    void run()
    {
        cv::namedWindow("Image Saver");
        cv::setMouseCallback("Image Saver", &ImageSaver::mouseCallback, this);
        ros::spin();
    }

    static void mouseCallback(int event, int x, int y, int flags, void* userdata)
    {
        // Do nothing
    }

private:
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    ros::NodeHandle nh_;
    message_filters::Subscriber<sensor_msgs::Image> image1_sub_;
    message_filters::Subscriber<sensor_msgs::Image> image2_sub_;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    std::shared_ptr<Sync> sync_;
    std::string save_dir_;
    int image_idx_;
    bool save_images_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_saver_node");
    ImageSaver image_saver;
    image_saver.run();
    return 0;
}
