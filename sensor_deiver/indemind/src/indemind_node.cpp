#include <fcntl.h>
#include <linux/videodev2.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <queue>
// #include "./indemind/include/DriverInterface.h"
#include "../include/DriverInterface.h"

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Header.h>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>

static const double PI = 3.14159265358979323846;
static const double GRAVITY = 9.8015;
double rad = PI / 180.;

using namespace indem;
ros::Publisher pub_imu;
ros::Publisher pub_image_l;
ros::Publisher pub_image_r;
bool imu_init_flag = false;
bool image_init_flag = false;
ros::Time imu_start_time;
ros::Time image_start_time;
bool show_falg = false;
int image_l_count = 0;
int image_r_count = 0;

cv::Rect rec_l(cv::Point2i(0, 0), cv::Point2i(640, 400));
cv::Rect rec_r(cv::Point2i(640, 0), cv::Point2i(1280, 400));
sensor_msgs::Imu imu_data;

void ImuCallBackFunction(IMUData *data) {
  // 打印imu数据 时间(ms) ACC（g） GYR(d / s)
  if (imu_init_flag == false) {
    imu_init_flag = true;
    imu_start_time = ros::Time::now() - ros::Duration(data->_timeStamp / 1000.);
  }

  imu_data.header.stamp = imu_start_time + ros::Duration(data->_timeStamp / 1000.);
  // imu_data.header.stamp = ros::Time::now();
  // imu_data.header.frame_id = "base_link";
  imu_data.angular_velocity.x = -(float)data->_gyr[0] * rad;
  imu_data.angular_velocity.y = -(float)data->_gyr[1] * rad;
  imu_data.angular_velocity.z = (float)data->_gyr[2] * rad;
  imu_data.linear_acceleration.x = -(float)data->_acc[0] * GRAVITY;
  imu_data.linear_acceleration.y = -(float)data->_acc[1] * GRAVITY;
  imu_data.linear_acceleration.z = (float)data->_acc[2] * GRAVITY;
  pub_imu.publish(imu_data);
  return;
}

void CameraCallbackFunction(cameraData *data) {
  //打印每帧图像时间
  // printf("%f\n", data->_timeStamp);
  cv::Mat img(data->_height, data->_width, CV_8UC1, data->_image);
  if (show_falg == true) {
    cv::imshow("camera", img);
    if (cv::waitKey(2) == 27) {  // esc 退出
      printf("esc ...\n");
      exit(0);
    }
  }

  // cv::Rect rec_l(cv::Point2i(0, 0), cv::Point2i(img.cols / 2, img.rows));
  // cv::Rect rec_r(cv::Point2i(img.cols / 2, 0), cv::Point2i(img.cols, img.rows));

  cv::Mat img_l = img(rec_l).clone();
  cv::Mat img_r = img(rec_r).clone();

  if (image_init_flag == false) {
    image_init_flag = true;
    image_start_time = ros::Time::now() - ros::Duration(data->_timeStamp / 1000.);
  }
  cv_bridge::CvImage cvi_l;
  sensor_msgs::Image imgl;
  cvi_l.header.stamp = image_start_time + ros::Duration(data->_timeStamp / 1000.);
  // cvi_l.header.stamp = ros::Time::now();
  cvi_l.header.seq = image_l_count++;
  cvi_l.encoding = "mono8";
  cvi_l.image = img_l;
  cvi_l.toImageMsg(imgl);
  pub_image_l.publish(imgl);

  cv_bridge::CvImage cvi_r;
  sensor_msgs::Image imgr;
  cvi_r.header.stamp = image_start_time + ros::Duration(data->_timeStamp / 1000.);
  // cvi_r.header.stamp = ros::Time::now();
  cvi_r.header.seq = image_r_count++;
  cvi_r.encoding = "mono8";
  cvi_r.image = img_r;
  cvi_r.toImageMsg(imgr);
  pub_image_r.publish(imgr);

  img_l.release();
  img_l.release();
  img.release();
  return;
}

void HMDHotplugCallback_func(bool bArrive) {
  if (bArrive) {
    printf("设备插入\n");
  } else {
    printf("设备移除\n");
  }
  return;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "indemind");
  ros::NodeHandle n("~");

  int ret = 0;
  int width = 0;         //单图像宽度 640 1280
  int height = 0;        //单图像高  400 800
  int fps = 0;           //帧率 25 50 100 200
  int imufreq = 0;       // imu频率
  int version = 255;     //代码固件版本号，初始值任意
  size_t info_size = 0;  //获取到的参数总长度
  unsigned char *module_info = new unsigned char[FLASH_MAX_SIZE];
  ModuleParamInFlash<1> moddule_param = {0};  //标定参数等信息等
  IDriverInterface *driver = DriverFactory();
  enum IMAGE_RESOLUTION plan = RESOLUTION_640;
  if (argc == 2) {
    show_falg = argv[1];  // treue  开机显示双目图像
  }

  if (argc == 5) {
    width = atoi(argv[1]);
    height = atoi(argv[2]);
    fps = atoi(argv[3]);
    imufreq = atoi(argv[4]);
    if (width == 1280)
      plan = RESOLUTION_1280;
    else if (width == 640)
      plan = RESOLUTION_640;
    else {
      width = 640;
      plan = RESOLUTION_640;
    }

    if (fps != 25 && fps != 50 && fps != 100 && fps != 200) {
      fps = 50;
    }
    if (fps == 200 && plan != RESOLUTION_640) {
      fps = 100;
    }

    if (imufreq > 1000) {
      imufreq = 1000;
    }
  } else {
    width = 1280;
    height = 800;
    fps = 25;
    imufreq = 1000;
  }
  //获取标定参数等信息等
  ret = driver->GetModuleParams(version, module_info, info_size);
  if (ret != true) {
    printf("Get params faild\n");
  } else {
    memcpy(&moddule_param, module_info, info_size);
  }
  //打开设备
  ret = driver->Open(imufreq, fps, plan);
  ret = 0;
  if (ret < 0) {
    printf("Open device err\n");
  } else {
    printf("Indemiind camera start ... \n ");
    driver->SetCameraCallback(CameraCallbackFunction);
    driver->SetIMUCallback(ImuCallBackFunction);
    SetHotplugCallback(HMDHotplugCallback_func);
  }

  pub_imu = n.advertise<sensor_msgs::Imu>("/indemind/imu", 20);
  pub_image_l = n.advertise<sensor_msgs::Image>("/indemind/left/image", 5);
  pub_image_r = n.advertise<sensor_msgs::Image>("/indemind/right/image", 5);

  ros::spin();
  return 0;
}
