
#include <iostream>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Header.h>
#include <std_srvs/SetBool.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/client.h>
#include "camera_player/configConfig.h"

// #include <image_geometry/pinhole_camera_model.h>

#include <thread>
#include <mutex>
#include <queue>

#include "tic_toc.h"
using namespace std;
using namespace cv;

ros::Publisher pub_rect_img;
ros::Publisher pub_raw_img;
ros::Publisher pub_cam_info;

string config_file;
VideoCapture capture;

double dynamic_brightness = 0, dynamic_contrast = 0, dynamic_saturation = 0,
       dynamic_hue = 0, dynamic_set_camera = 0, dynamic_fps = 60, dynamic_expose, dynamic_gain;

bool isLocalizationStart = true;
bool set_start(std_srvs::SetBool::Request &req,
               std_srvs::SetBool::Response &res)
{
    if (req.data > 0)
    {
        isLocalizationStart = true;
        res.message = "Enable localization!";
    }
    else
    {
        isLocalizationStart = false;
        res.message = "Disable localization!";
    }

    ROS_INFO("request: %d", req.data);
    ROS_INFO("sending back response: set localization %d", isLocalizationStart);
    res.success = true;
    return true;
}

void callback(camera_player::configConfig &config, uint32_t level)
{

    if (config.set_camera)
    {
        ROS_INFO(" config.set_camera");

        // if (!capture.set(CV_CAP_PROP_FRAME_WIDTH, config.image_height) &&
        //     !capture.set(CV_CAP_PROP_FRAME_HEIGHT, config.image_height))
        //     ROS_ERROR("unsupported imageSize.width, imageSize.height");

        // if (!capture.set(CV_CAP_PROP_FPS, config.fps))
        // {
        //     ROS_ERROR("unsupported  fPS  ");
        // }

        if (!capture.set(CV_CAP_PROP_BRIGHTNESS, config.brightness))
            ROS_ERROR("unsupported CV_CAP_PROP_BRIGHTNESS ");

        if (!capture.set(CV_CAP_PROP_EXPOSURE, config.expose))
            ROS_ERROR("unsupported CV_CAP_PROP_EXPOSURE ");

        if (!capture.set(CV_CAP_PROP_GAIN, config.gain))
            ROS_ERROR("unsupported CV_CAP_PROP_EXPOSURE ");

        if (!capture.set(CV_CAP_PROP_CONTRAST, config.contrast))
            ROS_ERROR("unsupported CV_CAP_PROP_CONTRAST ");

        // printf("width :      %.2f\n", capture.get(CV_CAP_PROP_FRAME_WIDTH));
        // printf("height :     %.2f\n", capture.get(CV_CAP_PROP_FRAME_HEIGHT));
        // printf("fbs :        %.2f\n", capture.get(CV_CAP_PROP_FPS));        //帧率 120
        printf("brightness : %.2f\n", capture.get(CV_CAP_PROP_BRIGHTNESS)); //亮度
        printf("contrast :   %.2f\n", capture.get(CV_CAP_PROP_CONTRAST));   //对比度  0.32
        printf("saturation : %.2f\n", capture.get(CV_CAP_PROP_SATURATION)); //饱和度 0.64
        printf("hue :        %.2f\n", capture.get(CV_CAP_PROP_HUE));        //色调  0.5
        printf("exposure :   %.2f\n", capture.get(CV_CAP_PROP_EXPOSURE));   //曝光 inf
        printf("gain :       %.2f\n", capture.get(CV_CAP_PROP_GAIN));       //曝光 inf
    }
}

void process()
{
    cv::Mat cameraMatrix, distCoeffs, rectification, projection;
    cv::Size imageSize;
    int rect = 0;
    int fps = 60;

    int device = -1;

    // std::string config_file_path= ros::package::getPath("image_rectify");
    FileStorage readfs;
    if (readfs.open(config_file, FileStorage::READ) == false)
    {
        ROS_ERROR("Cannot open the file.");
    }
    readfs["device"] >> device;
    readfs["rect"] >> rect;
    // readfs["brightness"] >> brightness;
    // readfs["contrast"] >> contrast;
    // readfs["saturation"] >> saturation;
    // readfs["set_camera"] >> set_camera;

    readfs["camera_matrix"] >> cameraMatrix;
    readfs["distortion_coefficients"] >> distCoeffs;
    readfs["image_width"] >> imageSize.width;
    readfs["image_height"] >> imageSize.height;
    readfs["fps"] >> fps;
    readfs["rectification_matrix"] >> rectification;

    readfs["projection_matrix"] >> projection;

    cout << cameraMatrix << endl
         << distCoeffs << endl
         << imageSize << endl
         << rectification << endl
         << projection << endl;
    readfs.release();

    bool do_rect = true;

    cv::Mat map1, map2, new_cameraMatrix;
    if (do_rect)
    {
        new_cameraMatrix = getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize); //返回新的内参矩阵
        initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), new_cameraMatrix, imageSize, CV_8UC1, map1, map2);
    }
    else
    {
        new_cameraMatrix = cameraMatrix;
    }
    sensor_msgs::CameraInfo cam_info;
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
    }
    cam_info.P[0] = new_cameraMatrix.at<double>(0);
    cam_info.P[1] = new_cameraMatrix.at<double>(1);
    cam_info.P[2] = new_cameraMatrix.at<double>(2);
    cam_info.P[3] = 0;
    cam_info.P[4] = new_cameraMatrix.at<double>(3);
    cam_info.P[5] = new_cameraMatrix.at<double>(4);
    cam_info.P[6] = new_cameraMatrix.at<double>(5);
    cam_info.P[7] = 0;
    cam_info.P[8] = new_cameraMatrix.at<double>(6);
    cam_info.P[9] = new_cameraMatrix.at<double>(7);
    cam_info.P[10] = new_cameraMatrix.at<double>(8);
    cam_info.P[11] = 0;
    cam_info.D.resize(distCoeffs.rows * distCoeffs.cols);
    for (int i = 0; i < distCoeffs.rows * distCoeffs.cols; i++)
    {
        cam_info.D[i] = distCoeffs.at<double>(i);
    }
    cam_info.binning_x = 1;
    cam_info.binning_y = 1;
    // model_.fromCameraInfo(cam_info);
    if (!capture.open(device))
    {
        ROS_ERROR("video not open.");
        return;
    }
    if (!capture.set(CV_CAP_PROP_AUTO_EXPOSURE, 0.25))
    {

        ROS_ERROR("unsupported CV_CAP_PROP_AUTO_EXPOSURE ");
    }
    if (!capture.set(CV_CAP_PROP_FRAME_WIDTH, imageSize.width) &&
        !capture.set(CV_CAP_PROP_FRAME_HEIGHT, imageSize.height))
    {

        ROS_ERROR("unsupported %d x %d ", imageSize.width, imageSize.height);
    }

    std::cout << "expect fps is " << fps << std::endl;
    if (!capture.set(CV_CAP_PROP_FPS, fps))
    {
        ROS_ERROR("unsupported  fPS  %d", fps);
    }
    printf("width :      %.2f\n", capture.get(CV_CAP_PROP_FRAME_WIDTH));
    printf("height :     %.2f\n", capture.get(CV_CAP_PROP_FRAME_HEIGHT));
    printf("fbs :        %.2f\n", capture.get(CV_CAP_PROP_FPS));        //帧率 120
    printf("brightness : %.2f\n", capture.get(CV_CAP_PROP_BRIGHTNESS)); //亮度
    printf("contrast :   %.2f\n", capture.get(CV_CAP_PROP_CONTRAST));   //对比度  0.32
    printf("saturation : %.2f\n", capture.get(CV_CAP_PROP_SATURATION)); //饱和度 0.64
    printf("hue :        %.2f\n", capture.get(CV_CAP_PROP_HUE));        //色调  0.5
    printf("exposure :   %.2f\n", capture.get(CV_CAP_PROP_EXPOSURE));   //曝光 inf
    ROS_INFO("play_opencv_node process  ....");

    cv::Mat image_raw, image_rec;
    while (1)
    {
        if (isLocalizationStart)
        {
            if (!capture.read(image_raw)) //1080p 50%
            {
                continue;
                ROS_ERROR("capture read erro ");
            }
            if (rect)
            {
                // ros::Time t0 = ros::Time::now();
                cv::remap(image_raw, image_rec, map1, map2, INTER_LINEAR); //3ms
                // ROS_INFO("rect time is %f" , (ros::Time::now() - t0).toSec());
            }
            else
                image_rec = image_raw;

            sensor_msgs::ImagePtr msg_rect = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_rec).toImageMsg(); //rgb
            sensor_msgs::ImagePtr msg_raw = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_raw).toImageMsg();

            msg_rect->header.stamp = ros::Time::now();
            msg_raw->header = msg_rect->header;
            cam_info.header = msg_rect->header;
            pub_raw_img.publish(msg_raw);
            pub_rect_img.publish(msg_rect);
            pub_cam_info.publish(cam_info);
        }
        chrono::milliseconds dura(10);
        this_thread::sleep_for(dura);
    }
    capture.release();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "play_opencv_node");
    ros::NodeHandle nh;

    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    if (argc != 2)
    {
        printf("please intput: rosrun image_rectify image_rectify_node [config file] \n"
               "for example: rosrun image_rectify image_rectify_node "
               "/home/jon/ubt_code/U40/tag_ws/src/image_rectify/config/config_1080.yaml \n");
        return 1;
    }
    config_file = argv[1];
    std::cout << "config_file: " << config_file << std::endl;

    nh.getParam("start_localization", isLocalizationStart);
    // ros::param::get("~start_localization" , isLocalizationStart , true);
    std::cout << "start_localization is " << isLocalizationStart << std::endl;

    ros::ServiceServer service = nh.advertiseService("start_localization", set_start);

    pub_rect_img = nh.advertise<sensor_msgs::Image>("/usb_cam/image_rect", 20);
    pub_raw_img = nh.advertise<sensor_msgs::Image>("/usb_cam/image_raw", 20);
    pub_cam_info = nh.advertise<sensor_msgs::CameraInfo>("/usb_cam/camera_info", 20);

    dynamic_reconfigure::Server<camera_player::configConfig> server;
    dynamic_reconfigure::Server<camera_player::configConfig>::CallbackType f;

    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    thread sync_thread{process};

    ros::spin();
    return 0;
}
