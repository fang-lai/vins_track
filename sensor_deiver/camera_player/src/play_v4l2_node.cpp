#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <queue>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>
#include <thread>

#include <V4L2Capture.hpp>
#include <tic_toc.h>

using namespace std;
using namespace cv;
#define IMAGEWIDTH 1920
#define IMAGEHEIGHT 1080

ros::Publisher pub_rect_img;
ros::Publisher pub_raw_img;
ros::Publisher pub_cam_info;
string config_file;

void play_process()
{
    cv::Mat cameraMatrix, distCoeffs, rectification, projection;
    cv::Size imageSize;
    string videoDev;

    FileStorage readfs;
    if (readfs.open(config_file, FileStorage::READ) == false) {
        ROS_ERROR("Cannot open the file.");
    }
    readfs["device"] >> videoDev;
    readfs["camera_matrix"] >> cameraMatrix;
    readfs["distortion_coefficients"] >> distCoeffs;
    readfs["image_width"] >> imageSize.width;
    readfs["image_height"] >> imageSize.height;
    readfs["rectification_matrix"] >> rectification;
    readfs["projection_matrix"] >> projection;
    readfs.release();
    cout << "\ncameraMatrix : \n"
         << cameraMatrix << endl;
    cout << "\ndistCoeffs :  \n"
         << distCoeffs << endl;
    cout << "\nimageSize :  \n"
         << imageSize << endl;
    cout << "\nrectification :  \n"
         << rectification << endl;
    cout << "\nprojection :  \n"
         << projection << "\n\n ---------------V4L2----------------\n"
         << endl;

    sensor_msgs::CameraInfo cam_info;
    cam_info.width = imageSize.width;
    cam_info.height = imageSize.height;
    cam_info.distortion_model = "plumb_bob";
    cam_info.roi.width = imageSize.width;
    cam_info.roi.height = imageSize.height;

    cv::Mat map1, map2, new_cameraMatrix;
    // new_cameraMatrix = getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 0, imageSize, 0);  //返回新的内参矩阵
    new_cameraMatrix = getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0);  //返回新的内参矩阵
    initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), new_cameraMatrix, imageSize, CV_8UC1, map1, map2);

    for (int i = 0; i < 9; i++) {
        cam_info.R[i] = rectification.at<double>(i);
    }
    for (int i = 0; i < 9; i++) {
        cam_info.K[i] = new_cameraMatrix.at<double>(i);
        // cam_info.K[i]=cameraMatrix.at<double>(i);
    }
    for (int i = 0; i < 12; i++) {
        cam_info.P[i] = projection.at<double>(i);
    }
    cam_info.D.resize(8);
    for (int i = 0; i < 8; i++) {
        cam_info.D[i] = distCoeffs.at<double>(i);
    }
    cam_info.binning_x = 1;
    cam_info.binning_y = 1;

    V4L2Capture* vcap = new V4L2Capture(const_cast<char*>(videoDev.c_str()), imageSize.width, imageSize.height);
    vcap->openDevice();
    vcap->initDevice();
    vcap->startCapture();

    pthread_setname_np(pthread_self(), "play_process");

    cv::Mat image_raw, image_rec;
    double t;
    unsigned char* yuv422frame = NULL;
    unsigned long yuvframeSize = 0;

    ROS_INFO("play_v4l2_node process  ....");
    while (1) {
        // t = (double)cvGetTickCount();
        vcap->getFrame((void**)&yuv422frame, (size_t*)&yuvframeSize);
        CvMat cvmat = cvMat(imageSize.height, imageSize.width, CV_8UC1, (void*)yuv422frame);  //CV_8UC3

        IplImage* img = cvDecodeImage(&cvmat, 1);  //解码
        if (!img) {
            ROS_ERROR("DecodeImage error!");
        }
        image_raw = cvarrToMat(img).clone();
        vcap->backFrame();
        cvReleaseImage(&img);

        // t = (double)cvGetTickCount() - t;
        // printf("Used time is %g ms\n",( t / (cvGetTickFrequency()*1000)));

        // cv::remap(image_raw, image_rec, map1, map2, INTER_LINEAR);  //3ms
        // image_rec=image_raw;
        // cv::imshow("image_raw", image_raw);
        // cv::imshow("image_rec", image_rec);
        // if (cv::waitKey(5) == 'q') {
        //     ROS_INFO("exit ....");
        //     vcap->stopCapture();
        //     vcap->freeBuffers();
        //     vcap->closeDevice();
        //     exit(0);
        // }
        if (image_raw.cols != imageSize.width && image_raw.rows != imageSize.height) {
            ROS_ERROR_STREAM("image raw image size error!" << image_raw.size());
        }

        sensor_msgs::ImagePtr msg_rect = cv_bridge::CvImage(std_msgs::Header(), "rgb8", image_raw).toImageMsg();
        msg_rect->header.stamp = ros::Time::now();
        pub_rect_img.publish(msg_rect);

        sensor_msgs::ImagePtr msg_raw = cv_bridge::CvImage(std_msgs::Header(), "rgb8", image_raw).toImageMsg();
        msg_raw->header = msg_rect->header;
        pub_raw_img.publish(msg_raw);

        cam_info.header = msg_rect->header;
        pub_cam_info.publish(cam_info);
        chrono::milliseconds dura(20);
        this_thread::sleep_for(dura);
    }
    vcap->stopCapture();
    vcap->freeBuffers();
    vcap->closeDevice();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "play_v4l2_node");
    ros::NodeHandle nh;

    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    if (argc != 2) {
        ROS_WARN("please intput: rosrun image_rectify image_rectify_node [config file] \n"
                 "for example: rosrun image_rectify image_rectify_node "
                 "/home/jon/ubt_code/U40/tag_ws/src/image_rectify/config/config_1080.yaml \n");
        return 1;
    }
    config_file = argv[1];
    std::cout << "config_file: " << config_file << std::endl;

    pub_rect_img = nh.advertise<sensor_msgs::Image>("/usb_cam/image_rect", 20);
    pub_raw_img = nh.advertise<sensor_msgs::Image>("/usb_cam/image_raw", 20);
    pub_cam_info = nh.advertise<sensor_msgs::CameraInfo>("/usb_cam/camera_info", 20);

    thread sync_thread{play_process};
    pthread_setname_np(pthread_self(), "video_player");

    ros::spin();
    return 0;
}