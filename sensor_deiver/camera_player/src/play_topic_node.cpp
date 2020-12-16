
#include <iostream>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

#include "tic_toc.h"

#include <thread>
#include <mutex>
#include <queue>

#include "tic_toc.h"
using namespace std;
using namespace cv;

ros::Publisher pub_rect_img;
ros::Publisher pub_raw_img;
ros::Publisher pub_cam_info;

string topic_name;
cv::Mat cameraMatrix, distCoeffs, rectification, projection;
cv::Size imageSize;
int fps = 60;
string config_file;

cv::Mat map1, map2, new_cameraMatrix;
sensor_msgs::CameraInfo cam_info;

void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    // TicToc img_time;
    cv_bridge::CvImageConstPtr ptr;

    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    // else if (img_msg->encoding == "rgb8")
    // {
    //     sensor_msgs::Image img;
    //     img.header = img_msg->header;
    //     img.height = img_msg->height;
    //     img.width = img_msg->width;
    //     img.is_bigendian = img_msg->is_bigendian;
    //     img.step = img_msg->step;
    //     img.data = img_msg->data;
    //     img.encoding = "rgb8";
    //     ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::RGB8);
    // }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
    cv::Mat image_raw = ptr->image.clone();

    cv::Mat image_rec;
    cv::remap(image_raw, image_rec, map1, map2, INTER_LINEAR); //3ms
    // image_rec = image_raw;
    // if (image_rec.type() == CV_8UC1) // 灰度图像
    // {
    //     cv::cvtColor(image_rec, image_rec, COLOR_GRAY2RGB); //变成彩色格式会对显示造成影响
    //     std::cout << "moon" << std::endl;
    // }

    // cv::imshow("image_raw", image_raw);
    // cv::imshow("image_rec", image_rec);
    // cv::waitKey(5);

    sensor_msgs::ImagePtr msg_rect = cv_bridge::CvImage(std_msgs::Header(), "mono8", image_rec).toImageMsg();
    // sensor_msgs::ImagePtr msg_raw = cv_bridge::CvImage(std_msgs::Header(), "rgb8", image_raw).toImageMsg();
    msg_rect->header.stamp = ros::Time::now();
    // msg_raw->header = msg_rect->header;
    cam_info.header = msg_rect->header;
    // pub_raw_img.publish(msg_raw);
    pub_rect_img.publish(msg_rect);
    pub_cam_info.publish(cam_info);

    // std::cout<<"image process time cost: "<<img_time.toc()<<std::endl; //< 2ms
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "play_topic_node");
    ros::NodeHandle nh;

    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    if (argc != 2)
    {
        printf("please intput: rosrun video_player play_topic_node [config file] \n"
               "for example: rosrun video_player play_topic_node "
               "/home/jon/ubt_code/U40/tag_ws/src/video_player/config/config_1080.yaml \n");
        return 1;
    }
    config_file = argv[1];
    std::cout << "config_file: " << config_file << std::endl;

    //read config file
    FileStorage readfs;
    if (readfs.open(config_file, FileStorage::READ) == false)
    {
        ROS_ERROR("Cannot open the file.");
    }
    readfs["topic"] >> topic_name;
    readfs["camera_matrix"] >> cameraMatrix;
    readfs["distortion_coefficients"] >> distCoeffs;
    readfs["image_width"] >> imageSize.width;
    readfs["image_height"] >> imageSize.height;
    readfs["rectification_matrix"] >> rectification;
    readfs["projection_matrix"] >> projection;
    cout << cameraMatrix << endl
         << distCoeffs << endl
         << imageSize << endl
         << rectification << endl
         << projection << endl;
    readfs.release();

    new_cameraMatrix = getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 0, imageSize,0); 
    initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), new_cameraMatrix, imageSize, CV_8UC1, map1, map2);

    cam_info.width = imageSize.width;
    cam_info.height = imageSize.height;
    cam_info.distortion_model = "plumb_bob";
    cam_info.roi.width = imageSize.width;
    cam_info.roi.height = imageSize.height;

    for (int i = 0; i < 9; i++)
    {
        cam_info.R[i] = rectification.at<double>(i);
    }
    for (int i = 0; i < 9; i++)
    {
        cam_info.K[i] = new_cameraMatrix.at<double>(i);
        // cam_info.K[i]=cameraMatrix.at<double>(i);
    }
    for (int i = 0; i < 12; i++)
    {
        cam_info.P[i] = projection.at<double>(i);
    }
    cam_info.D.resize(8);
    for (int i = 0; i < 8; i++)
    {
        cam_info.D[i] = distCoeffs.at<double>(i);
    }
    cam_info.binning_x = 1;
    cam_info.binning_y = 1;

    ROS_INFO("play_topic_node process  ....");

    ros::Subscriber sub_img = nh.subscribe(topic_name, 20, img_callback);

    pub_rect_img = nh.advertise<sensor_msgs::Image>("/usb_cam/image_rect", 20);
    // pub_raw_img = nh.advertise<sensor_msgs::Image>("/usb_cam/image_raw", 20);
    pub_cam_info = nh.advertise<sensor_msgs::CameraInfo>("/usb_cam/camera_info", 20);

    ros::spin();
    return 0;
}