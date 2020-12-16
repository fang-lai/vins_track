/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science
 *and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

// #include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <ros/ros.h>
#include <stdio.h>
#include <map>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <queue>
#include <thread>
#include "estimator/estimator.h"
#include "estimator/parameters.h"
#include "utility/visualization.h"

Estimator estimator;

std::queue<sensor_msgs::ImuConstPtr> imu_buf;
std::vector<std::queue<sensor_msgs::ImageConstPtr>> imgs_buf;
// queue<sensor_msgs::PointCloudConstPtr> feature_buf;
std::mutex m_buf;

void ImgCallBackOne(const sensor_msgs::ImageConstPtr &img0_msg) {
  m_buf.lock();
  imgs_buf[0].push(img0_msg);
  m_buf.unlock();
}
void ImgCallBackTwo(const sensor_msgs::ImageConstPtr &img0_msg,
                    const sensor_msgs::ImageConstPtr &img1_msg) {
  m_buf.lock();
  imgs_buf[0].push(img0_msg);
  imgs_buf[1].push(img1_msg);
  m_buf.unlock();
}
void ImgCallBackThree(const sensor_msgs::ImageConstPtr &img0_msg,
                      const sensor_msgs::ImageConstPtr &img1_msg,
                      const sensor_msgs::ImageConstPtr &img2_msg) {
  m_buf.lock();
  imgs_buf[0].push(img0_msg);
  imgs_buf[1].push(img1_msg);
  imgs_buf[2].push(img2_msg);
  m_buf.unlock();
}
void ImgCallBackFour(const sensor_msgs::ImageConstPtr &img0_msg,
                     const sensor_msgs::ImageConstPtr &img1_msg,
                     const sensor_msgs::ImageConstPtr &img2_msg,
                     const sensor_msgs::ImageConstPtr &img3_msg) {
  m_buf.lock();
  imgs_buf[0].push(img0_msg);
  imgs_buf[1].push(img1_msg);
  imgs_buf[2].push(img2_msg);
  imgs_buf[3].push(img3_msg);
  m_buf.unlock();
}

void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg) {
  double t = imu_msg->header.stamp.toSec();
  double dx = imu_msg->linear_acceleration.x;
  double dy = imu_msg->linear_acceleration.y;
  double dz = imu_msg->linear_acceleration.z;
  double rx = imu_msg->angular_velocity.x;
  double ry = imu_msg->angular_velocity.y;
  double rz = imu_msg->angular_velocity.z;
  Vector3d acc(dx, dy, dz);
  Vector3d gyr(rx, ry, rz);
  estimator.inputIMU(t, acc, gyr);
  return;
}

cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg) {
  cv_bridge::CvImageConstPtr ptr;
  if (img_msg->encoding == "8UC1") {
    sensor_msgs::Image img;
    img.header = img_msg->header;
    img.height = img_msg->height;
    img.width = img_msg->width;
    img.is_bigendian = img_msg->is_bigendian;
    img.step = img_msg->step;
    img.data = img_msg->data;
    img.encoding = "mono8";
    ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
  } else
    ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

  cv::Mat img = ptr->image.clone();
  return img;
}

// extract images with same timestamp from two topics
void sync_process() {
  while (1) {
    m_buf.lock();
    if (!imgs_buf[0].empty()) {
      double time = imgs_buf[0].front()->header.stamp.toSec();
      std::queue<std::pair<int, cv::Mat>> id_image_buf;
      for (int i = 0; i < NUM_OF_CAM; i++) {
        id_image_buf.push(
            std::make_pair(i, getImageFromMsg(imgs_buf[i].front())));
        imgs_buf[i].pop();
      }
      // TicToc estimate_time;
      estimator.inputImage(time, id_image_buf);
    }
    m_buf.unlock();
    std::chrono::milliseconds dura(2);
    std::this_thread::sleep_for(dura);
  }
}

/*
// void feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg)
// {
//     map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
//     for (unsigned int i = 0; i < feature_msg->points.size(); i++)
//     {
//         int feature_id = feature_msg->channels[0].values[i];
//         int camera_id = feature_msg->channels[1].values[i];
//         double x = feature_msg->points[i].x;
//         double y = feature_msg->points[i].y;
//         double z = feature_msg->points[i].z;
//         double p_u = feature_msg->channels[2].values[i];
//         double p_v = feature_msg->channels[3].values[i];
//         double velocity_x = feature_msg->channels[4].values[i];
//         double velocity_y = feature_msg->channels[5].values[i];
//         if(feature_msg->channels.size() > 5)
//         {
//             double gx = feature_msg->channels[6].values[i];
//             double gy = feature_msg->channels[7].values[i];
//             double gz = feature_msg->channels[8].values[i];
//             pts_gt[feature_id] = Eigen::Vector3d(gx, gy, gz);
//             //printf("receive pts gt %d %f %f %f\n", feature_id, gx, gy, gz);
//         }
//         ROS_ASSERT(z == 1);
//         Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
//         xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
//         featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
//     }
//     double t = feature_msg->header.stamp.toSec();
//     estimator.inputFeature(t, featureFrame);
//     return;
// }

// void restart_callback(const std_msgs::BoolConstPtr &restart_msg)
// {
//     if (restart_msg->data == true)
//     {
//         ROS_WARN("restart the estimator!");
//         estimator.clearState();
//         estimator.setParameter();
//     }
//     return;
// }

// void imu_switch_callback(const std_msgs::BoolConstPtr &switch_msg)
// {
//     if (switch_msg->data == true)
//     {
//         //ROS_WARN("use IMU!");
//         estimator.changeSensorType(1, STEREO);
//     }
//     else
//     {
//         //ROS_WARN("disable IMU!");
//         estimator.changeSensorType(0, STEREO);
//     }
//     return;
// }

// void cam_switch_callback(const std_msgs::BoolConstPtr &switch_msg)
// {
//     if (switch_msg->data == true)
//     {
//         //ROS_WARN("use stereo!");
//         estimator.changeSensorType(USE_IMU, 1);
//     }
//     else
//     {
//         //ROS_WARN("use mono camera (left)!");
//         estimator.changeSensorType(USE_IMU, 0);
//     }
//     return;
// }
*/
int main(int argc, char **argv) {
  ros::init(argc, argv, "vins_estimator");
  ros::NodeHandle nh("~");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                 ros::console::levels::Info);
  if (argc != 2) {
    printf(
        "please intput: rosrun vins vins_node [config file] \n"
        "for example: rosrun vins vins_node "
        "~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml "
        "\n");
    return 1;
  }
  string config_file = argv[1];
  printf("config_file: %s\n", argv[1]);
  readParameters(config_file);
  estimator.setParameter();
  int image_size = IMAGE_TOPIC.size();
  assert(image_size > 0);
  imgs_buf.resize(image_size);
  if (image_size == 1) {
    static ros::Subscriber img_sub =
        nh.subscribe(IMAGE_TOPIC[0], 20, ImgCallBackOne);
  }
  typedef message_filters::Subscriber<sensor_msgs::Image> ImgSubsriber;
  if (image_size == 2) {
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                            sensor_msgs::Image>
        SyncPolicy;
    static ImgSubsriber imgsub_0(nh, IMAGE_TOPIC[0], 20);
    static ImgSubsriber imgsub_1(nh, IMAGE_TOPIC[1], 20);
    static message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10),
                                                          imgsub_0, imgsub_1);
    sync.registerCallback(boost::bind(&ImgCallBackTwo, _1, _2));
  }
  if (image_size == 3) {
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image>
        SyncPolicy;
    static ImgSubsriber imgsub_0(nh, IMAGE_TOPIC[0], 20);
    static ImgSubsriber imgsub_1(nh, IMAGE_TOPIC[1], 20);
    static ImgSubsriber imgsub_2(nh, IMAGE_TOPIC[2], 20);
    static message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), imgsub_0,
                                                   imgsub_1, imgsub_2);
    sync.registerCallback(boost::bind(&ImgCallBackThree, _1, _2, _3));
  }
  if (image_size == 4) {
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image,
        sensor_msgs::Image>
        SyncPolicy;
    static ImgSubsriber imgsub_0(nh, IMAGE_TOPIC[0], 20);
    static ImgSubsriber imgsub_1(nh, IMAGE_TOPIC[1], 20);
    static ImgSubsriber imgsub_2(nh, IMAGE_TOPIC[2], 20);
    static ImgSubsriber imgsub_3(nh, IMAGE_TOPIC[3], 20);
    static message_filters::Synchronizer<SyncPolicy> sync(
        SyncPolicy(10), imgsub_0, imgsub_1, imgsub_2, imgsub_3);
    sync.registerCallback(boost::bind(&ImgCallBackFour, _1, _2, _3, _4));
  }

#ifdef EIGEN_DONT_PARALLELIZE
  ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif

  ROS_WARN("waiting for image and imu...");

  registerPub(nh);

  ros::Subscriber sub_imu = nh.subscribe(IMU_TOPIC, 2000, imu_callback,
                                         ros::TransportHints().tcpNoDelay());
  // ros::Subscriber sub_feature = n.subscribe("/feature_tracker/feature", 2000,
  // feature_callback); ros::Subscriber sub_restart =
  // n.subscribe("/vins_restart", 100, restart_callback); ros::Subscriber
  // sub_imu_switch = n.subscribe("/vins_imu_switch", 100, imu_switch_callback);
  // ros::Subscriber sub_cam_switch = n.subscribe("/vins_cam_switch", 100,
  // cam_switch_callback)
  std::thread sync_thread{sync_process};
  pthread_setname_np(sync_thread.native_handle(), "sync_thread");

  ros::spin();

  return 0;
}
