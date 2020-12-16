/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science
 *and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "parameters.h"

double INIT_DEPTH;
double MIN_PARALLAX;
double ACC_N, ACC_W;
double GYR_N, GYR_W;

std::vector<Eigen::Matrix3d> RIC;
std::vector<Eigen::Vector3d> TIC;

Eigen::Vector3d G{0.0, 0.0, 9.8};

double BIAS_ACC_THRESHOLD;
double BIAS_GYR_THRESHOLD;
double SOLVER_TIME;
int NUM_ITERATIONS;
int ESTIMATE_EXTRINSIC;
int ESTIMATE_TD;
int ROLLING_SHUTTER;
std::string EX_CALIB_RESULT_PATH;
std::string VINS_RESULT_PATH;
std::string OUTPUT_FOLDER;
std::string IMU_TOPIC;
int ROW, COL;
double TD;
int NUM_OF_CAM;
int NUM_OF_MONO_CAM;
int NUM_OF_STEREO_CAM;
int STEREO;
int USE_IMU;
int MULTIPLE_THREAD;
map<int, Eigen::Vector3d> pts_gt;
std::string IMAGE0_TOPIC, IMAGE1_TOPIC;
std::vector<std::string> IMAGE_TOPIC;
std::string FISHEYE_MASK;
int FISHEYE_MASK_RAD;
std::vector<std::string> CAM_NAMES;
int MAX_CNT;
int MIN_DIST;
double F_THRESHOLD;
int SHOW_TRACK;
int FLOW_BACK;

template <typename T>
T readParam(ros::NodeHandle &n, std::string name) {
  T ans;
  if (n.getParam(name, ans)) {
    ROS_INFO_STREAM("Loaded " << name << ": " << ans);
  } else {
    ROS_ERROR_STREAM("Failed to load " << name);
    n.shutdown();
  }
  return ans;
}

void readParameters(std::string config_file) {
  FILE *fh = fopen(config_file.c_str(), "r");
  if (fh == NULL) {
    ROS_WARN("config_file dosen't exist; wrong config_file path");
    ROS_BREAK();
    return;
  }
  fclose(fh);

  // paraments
  cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
  if (!fsSettings.isOpened()) {
    std::cerr << "ERROR: Wrong path to settings" << std::endl;
  }

  MAX_CNT = fsSettings["max_cnt"];
  MIN_DIST = fsSettings["min_dist"];
  F_THRESHOLD = fsSettings["F_threshold"];
  SHOW_TRACK = fsSettings["show_track"];
  FLOW_BACK = fsSettings["flow_back"];

  MULTIPLE_THREAD = fsSettings["multiple_thread"];

  USE_IMU = fsSettings["imu"];
  printf("USE_IMU: %d\n", USE_IMU);

  SOLVER_TIME = fsSettings["max_solver_time"];
  NUM_ITERATIONS = fsSettings["max_num_iterations"];
  MIN_PARALLAX = fsSettings["keyframe_parallax"];
  MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;

  fsSettings["output_path"] >> OUTPUT_FOLDER;
  VINS_RESULT_PATH = OUTPUT_FOLDER + "/vio.csv";
  std::cout << "result path " << VINS_RESULT_PATH << std::endl;
  std::ofstream fout(VINS_RESULT_PATH, std::ios::out);
  fout.close();

  INIT_DEPTH = 5.0;
  BIAS_ACC_THRESHOLD = 0.1;
  BIAS_GYR_THRESHOLD = 0.1;

  TD = fsSettings["td"];
  ESTIMATE_TD = fsSettings["estimate_td"];
  if (ESTIMATE_TD)
    ROS_INFO_STREAM(
        "Unsynchronized sensors, online estimate time offset, initial td: "
        << TD);
  else
    ROS_INFO_STREAM("Synchronized sensors, fix time offset: " << TD);

  // camera
  ROW = fsSettings["image_height"];
  COL = fsSettings["image_width"];
  ROS_INFO("ROW: %d COL: %d ", ROW, COL);

  NUM_OF_STEREO_CAM = fsSettings["num_of_stereo_cam"];
  NUM_OF_MONO_CAM = fsSettings["num_of_mono_cam"];
  NUM_OF_CAM = NUM_OF_MONO_CAM + NUM_OF_STEREO_CAM * 2;
  ROS_INFO("Stereo: %d Mono: %d ", NUM_OF_STEREO_CAM, NUM_OF_MONO_CAM);
  assert(NUM_OF_CAM > 0 && NUM_OF_CAM <= 4);

  FISHEYE_MASK_RAD = fsSettings["fish_eye_mask_radius"];
  fsSettings["fish_eye_mask_path"] >> FISHEYE_MASK;

  int pn = config_file.find_last_of('/');
  std::string configPath = config_file.substr(0, pn);
  ESTIMATE_EXTRINSIC = fsSettings["estimate_extrinsic"];
  for (int i = 0; i < NUM_OF_CAM; i++) {
    std::string image_topic_name = "image" + to_string(i) + "_topic";
    std::string image_topic;
    fsSettings[image_topic_name] >> image_topic;
    IMAGE_TOPIC.push_back(image_topic);

    std::cout << "camera " << i << " image_topic :" << image_topic << std::endl;

    std::string calib_file_name = "cam" + to_string(i) + "_calib";
    std::string calib_file;
    fsSettings[calib_file_name] >> calib_file;
    std::string camPath = configPath + "/" + calib_file;
    CAM_NAMES.push_back(camPath);
    if (ESTIMATE_EXTRINSIC == 2) {
      ROS_WARN(
          "have no prior about extrinsic param, calibrate extrinsic param");
      RIC.push_back(Eigen::Matrix3d::Identity());
      TIC.push_back(Eigen::Vector3d::Zero());
      EX_CALIB_RESULT_PATH = OUTPUT_FOLDER + "/extrinsic_parameter.csv";
    } else {
      if (ESTIMATE_EXTRINSIC == 1) {
        ROS_WARN(" Optimize extrinsic param around initial guess!");
        EX_CALIB_RESULT_PATH = OUTPUT_FOLDER + "/extrinsic_parameter.csv";
      }
      if (ESTIMATE_EXTRINSIC == 0) ROS_WARN(" fix extrinsic param ");

      cv::Mat cv_T;
      std::string extrinsic_name = "body_T_cam" + to_string(i);
      fsSettings[extrinsic_name] >> cv_T;
      Eigen::Matrix4d T;
      cv::cv2eigen(cv_T, T);
      RIC.push_back(T.block<3, 3>(0, 0));
      TIC.push_back(T.block<3, 1>(0, 3));
    }
  }

  // IMU
  if (USE_IMU) {
    fsSettings["imu_topic"] >> IMU_TOPIC;
    printf("IMU_TOPIC: %s\n", IMU_TOPIC.c_str());
    ACC_N = fsSettings["acc_n"];
    ACC_W = fsSettings["acc_w"];
    GYR_N = fsSettings["gyr_n"];
    GYR_W = fsSettings["gyr_w"];
    G.z() = fsSettings["g_norm"];
  } else {
    ESTIMATE_EXTRINSIC = 0;
    ESTIMATE_TD = 0;
    printf("no imu, fix extrinsic param; no time offset calibration\n");
  }
  fsSettings.release();
}
