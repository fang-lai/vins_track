/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include "pose_graph.h"

PoseGraph::PoseGraph() {
  posegraph_visualization = new CameraPoseVisualization(1.0, 0.0, 1.0, 1.0);
  posegraph_visualization->setScale(0.1);
  posegraph_visualization->setLineWidth(0.01);
  earliest_loop_index = -1;  //最先回环上的id
  t_drift = Eigen::Vector3d(0, 0, 0);
  yaw_drift = 0;
  r_drift = Eigen::Matrix3d::Identity();
  w_t_vio = Eigen::Vector3d(0, 0, 0);
  w_r_vio = Eigen::Matrix3d::Identity();
  global_index = 0;
  sequence_cnt = 0;
  sequence_loop.push_back(0);
  base_sequence = 1;
  use_imu = 0;
}

PoseGraph::~PoseGraph() { t_optimization.detach(); }

void PoseGraph::registerPub(ros::NodeHandle& n) {
  pub_pg_path = n.advertise<nav_msgs::Path>("pose_graph_path", 1000);
  pub_base_path = n.advertise<nav_msgs::Path>("base_path", 1000);
  pub_pose_graph = n.advertise<visualization_msgs::MarkerArray>("pose_graph", 1000);
  for (int i = 1; i < 10; i++) pub_path[i] = n.advertise<nav_msgs::Path>("path_" + to_string(i), 1000);
}

void PoseGraph::setIMUFlag(bool _use_imu) {
  use_imu = _use_imu;
  if (use_imu) {
    printf("VIO input, perfrom 4 DoF (x, y, z, yaw) pose graph optimization\n");
    t_optimization = std::thread(&PoseGraph::optimize4DoF, this);
  } else {
    printf("VO input, perfrom 6 DoF pose graph optimization\n");
    t_optimization = std::thread(&PoseGraph::optimize6DoF, this);
  }
}

void PoseGraph::loadVocabulary(std::string voc_path) {
  voc = new BriefVocabulary(voc_path);
  db.setVocabulary(*voc, false, 0);
}

void PoseGraph::addKeyFrame(KeyFrame* cur_kf, bool flag_detect_loop) {
  // shift to base frame
  Vector3d vio_P_cur;
  Matrix3d vio_R_cur;
  if (sequence_cnt != cur_kf->sequence) {  //重置回环
    sequence_cnt++;
    sequence_loop.push_back(0);
    w_t_vio = Eigen::Vector3d(0, 0, 0);
    w_r_vio = Eigen::Matrix3d::Identity();
    m_drift.lock();
    t_drift = Eigen::Vector3d(0, 0, 0);
    r_drift = Eigen::Matrix3d::Identity();
    m_drift.unlock();
  }

  cur_kf->getVioPose(vio_P_cur, vio_R_cur);   //获取当前位姿
  vio_P_cur = w_r_vio * vio_P_cur + w_t_vio;  //相对于世界坐标系位姿,由于可能会重新设置回环世界坐标有变化
  vio_R_cur = w_r_vio * vio_R_cur;
  cur_kf->updateVioPose(vio_P_cur, vio_R_cur);  //重新设置当前位姿
  cur_kf->index = global_index;                 //设置关键帧全局id 从0开始
  global_index++;
  int loop_index = -1;
  if (flag_detect_loop) {
    TicToc tmp_t;
    loop_index = detectLoop(cur_kf, cur_kf->index);  //回环检测 ,返回匹配的关键帧
  } else {
    addKeyFrameIntoVoc(cur_kf);  //不想检测回环则先将描述子添加到数据库
  }
  if (loop_index != -1) {  //检测到回环
    printf(" %d detect loop with %d \n", cur_kf->index, loop_index);
    KeyFrame* old_kf = getKeyFrame(loop_index);  //对应的关键帧
    // assert(old_kf != NULL);
    if (cur_kf->findConnection(old_kf)) {  // pnp 成功,old 2d , cur 3d
      if (earliest_loop_index > loop_index || earliest_loop_index == -1) earliest_loop_index = loop_index; //加载帧默认loop_index=-1,这里设置为新加的入的帧

      Vector3d w_P_old, w_P_cur, vio_P_cur;
      Matrix3d w_R_old, w_R_cur, vio_R_cur;
      old_kf->getVioPose(w_P_old, w_R_old);
      cur_kf->getVioPose(vio_P_cur, vio_R_cur);

      Vector3d relative_t;
      Quaterniond relative_q;
      relative_t = cur_kf->getLoopRelativeT();  // Tw1_bj
      relative_q = (cur_kf->getLoopRelativeQ()).toRotationMatrix();
      w_P_cur = w_R_old * relative_t + w_P_old;  //当前帧在回环坐标系下的位姿
      w_R_cur = w_R_old * relative_q;
      double shift_yaw;
      Matrix3d shift_r;
      Vector3d shift_t;
      if (use_imu) {                                                              //有imu 直接利用yaw角直接算出相对旋转 ??????
        shift_yaw = Utility::R2ypr(w_R_cur).x() - Utility::R2ypr(vio_R_cur).x();  //相对的yaw角
        shift_r = Utility::ypr2R(Vector3d(shift_yaw, 0, 0));                      // yaw转为旋转矩阵
      } else
        shift_r = w_R_cur * vio_R_cur.transpose();  //没yaw 利用整个旋转矩阵
      shift_t = w_P_cur - w_R_cur * vio_R_cur.transpose() * vio_P_cur;
      // shift vio pose of whole sequence to the world frame
      //回环可分成几段进行,没按'n' 应该是一样的
      if (old_kf->sequence != cur_kf->sequence && sequence_loop[cur_kf->sequence] == 0) {
        w_r_vio = shift_r;
        w_t_vio = shift_t;
        vio_P_cur = w_r_vio * vio_P_cur + w_t_vio;
        vio_R_cur = w_r_vio * vio_R_cur;
        cur_kf->updateVioPose(vio_P_cur, vio_R_cur);
        list<KeyFrame*>::iterator it = keyframelist.begin();
        for (; it != keyframelist.end(); it++) {
          if ((*it)->sequence == cur_kf->sequence) {
            Vector3d vio_P_cur;
            Matrix3d vio_R_cur;
            (*it)->getVioPose(vio_P_cur, vio_R_cur);
            vio_P_cur = w_r_vio * vio_P_cur + w_t_vio;
            vio_R_cur = w_r_vio * vio_R_cur;
            (*it)->updateVioPose(vio_P_cur, vio_R_cur);
          }
        }
        sequence_loop[cur_kf->sequence] = 1;
      }
      m_optimize_buf.lock();
      optimize_buf.push(cur_kf->index);  //放入buff中给优化器
      m_optimize_buf.unlock();
    } 
  }
  m_keyframelist.lock();
  Vector3d P;
  Matrix3d R;
  cur_kf->getVioPose(P, R);
  P = r_drift * P + t_drift;  //矫正当前帧位姿.
  R = r_drift * R;
  cur_kf->updatePose(P, R);
  Quaterniond Q{R};
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.stamp = ros::Time(cur_kf->time_stamp);
  pose_stamped.header.frame_id = "world";
  pose_stamped.pose.position.x = P.x() + VISUALIZATION_SHIFT_X;
  pose_stamped.pose.position.y = P.y() + VISUALIZATION_SHIFT_Y;
  pose_stamped.pose.position.z = P.z();
  pose_stamped.pose.orientation.x = Q.x();
  pose_stamped.pose.orientation.y = Q.y();
  pose_stamped.pose.orientation.z = Q.z();
  pose_stamped.pose.orientation.w = Q.w();
  path[sequence_cnt].poses.push_back(pose_stamped);
  path[sequence_cnt].header = pose_stamped.header;

  if (SAVE_LOOP_PATH) {
    ofstream loop_path_file(VINS_RESULT_PATH, ios::app);
    loop_path_file.setf(ios::fixed, ios::floatfield);
    loop_path_file.precision(0);
    loop_path_file << cur_kf->time_stamp * 1e9 << ",";
    loop_path_file.precision(5);
    loop_path_file << P.x() << "," << P.y() << "," << P.z() << "," << Q.w() << "," << Q.x() << "," << Q.y() << "," << Q.z() << "," << endl;
    loop_path_file.close();
  }
  // draw local connection
  if (SHOW_S_EDGE) {
    list<KeyFrame*>::reverse_iterator rit = keyframelist.rbegin();
    for (int i = 0; i < 4; i++) {
      if (rit == keyframelist.rend()) break;
      Vector3d conncected_P;
      Matrix3d connected_R;
      if ((*rit)->sequence == cur_kf->sequence) {
        (*rit)->getPose(conncected_P, connected_R);
        posegraph_visualization->add_edge(P, conncected_P);
      }
      rit++;
    }
  }
  if (SHOW_L_EDGE) {
    if (cur_kf->has_loop) {
      // printf("has loop \n");
      KeyFrame* connected_KF = getKeyFrame(cur_kf->loop_index);
      Vector3d connected_P, P0;
      Matrix3d connected_R, R0;
      connected_KF->getPose(connected_P, connected_R);
      // cur_kf->getVioPose(P0, R0);
      cur_kf->getPose(P0, R0);
      if (cur_kf->sequence > 0) {
        // printf("add loop into visual \n");
        posegraph_visualization->add_loopedge(P0, connected_P + Vector3d(VISUALIZATION_SHIFT_X, VISUALIZATION_SHIFT_Y, 0));
      }
    }
  }
  // posegraph_visualization->add_pose(P + Vector3d(VISUALIZATION_SHIFT_X, VISUALIZATION_SHIFT_Y, 0), Q);

  keyframelist.push_back(cur_kf);  //添加关键帧
  publish();
  m_keyframelist.unlock();
}

void PoseGraph::loadKeyFrame(KeyFrame* cur_kf, bool flag_detect_loop) {
  cur_kf->index = global_index;
  global_index++;
  int loop_index = -1;
  if (flag_detect_loop)
    loop_index = detectLoop(cur_kf, cur_kf->index);
  else {
    addKeyFrameIntoVoc(cur_kf);
  }
  if (loop_index != -1) {
    printf(" %d detect loop with %d \n", cur_kf->index, loop_index);
    KeyFrame* old_kf = getKeyFrame(loop_index);
    if (cur_kf->findConnection(old_kf)) {
      if (earliest_loop_index > loop_index || earliest_loop_index == -1) earliest_loop_index = loop_index;
      m_optimize_buf.lock();
      optimize_buf.push(cur_kf->index);  //放进优化器中
      m_optimize_buf.unlock();
    }
  }
  m_keyframelist.lock();
  Vector3d P;
  Matrix3d R;
  cur_kf->getPose(P, R);
  Quaterniond Q{R};
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.stamp = ros::Time(cur_kf->time_stamp);
  pose_stamped.header.frame_id = "world";
  pose_stamped.pose.position.x = P.x() + VISUALIZATION_SHIFT_X;
  pose_stamped.pose.position.y = P.y() + VISUALIZATION_SHIFT_Y;
  pose_stamped.pose.position.z = P.z();
  pose_stamped.pose.orientation.x = Q.x();
  pose_stamped.pose.orientation.y = Q.y();
  pose_stamped.pose.orientation.z = Q.z();
  pose_stamped.pose.orientation.w = Q.w();
  base_path.poses.push_back(pose_stamped);
  base_path.header = pose_stamped.header;
  // draw local connection
  if (SHOW_S_EDGE) {
    list<KeyFrame*>::reverse_iterator rit = keyframelist.rbegin();
    for (int i = 0; i < 1; i++) {
      if (rit == keyframelist.rend()) break;
      Vector3d conncected_P;
      Matrix3d connected_R;
      if ((*rit)->sequence == cur_kf->sequence) {
        (*rit)->getPose(conncected_P, connected_R);
        posegraph_visualization->add_edge(P, conncected_P);
      }
      rit++;
    }
  }
  /*
  if (cur_kf->has_loop)
  {
      KeyFrame* connected_KF = getKeyFrame(cur_kf->loop_index);
      Vector3d connected_P;
      Matrix3d connected_R;
      connected_KF->getPose(connected_P,  connected_R);
      posegraph_visualization->add_loopedge(P, connected_P, SHIFT);
  }
  */

  keyframelist.push_back(cur_kf);

  // publish();
  m_keyframelist.unlock();
}

KeyFrame* PoseGraph::getKeyFrame(int index) {
  //    unique_lock<mutex> lock(m_keyframelist);
  list<KeyFrame*>::iterator it = keyframelist.begin();
  for (; it != keyframelist.end(); it++) {
    if ((*it)->index == index) break;
  }
  if (it != keyframelist.end())
    return *it;
  else
    return NULL;
}

int PoseGraph::detectLoop(KeyFrame* keyframe, int frame_index) {
  // put image into image_pool; for visualization
  cv::Mat compressed_image;
  if (DEBUG_IMAGE) {
    int feature_num = keyframe->keypoints.size();
    cv::resize(keyframe->image, compressed_image, cv::Size(376, 240));
    putText(compressed_image, "feature_num:" + to_string(feature_num), cv::Point2f(10, 10), CV_FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255));
    image_pool[frame_index] = compressed_image;
  }
  TicToc tmp_t;
  // first query; then add this frame into database!
  QueryResults ret;  //查询结果
  TicToc t_query;
  db.query(keyframe->brief_descriptors, ret, 4, frame_index - 50);  //寻找4个最相似的帧 ,50帧之前的帧中找
  // printf("query time: %f", t_query.toc());
  // cout << "Searching for Image " << frame_index << ". " << ret << endl;

  TicToc t_add;
  // TODO youfang  添加进数据库loop
  if (ADD_DATA_BASE) db.add(keyframe->brief_descriptors);  //将当前帧加入到数据库中
  // printf("add feature time: %f", t_add.toc());
  // ret[0] is the nearest neighbour's score. threshold change with neighour score
  bool find_loop = false;
  cv::Mat loop_result;
  if (DEBUG_IMAGE) {
    loop_result = compressed_image.clone();
    if (ret.size() > 0) putText(loop_result, "neighbour score:" + to_string(ret[0].Score), cv::Point2f(10, 50), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255));
  }
  // visual loop result
  if (DEBUG_IMAGE) {
    for (unsigned int i = 0; i < ret.size(); i++) {
      int tmp_index = ret[i].Id;
      auto it = image_pool.find(tmp_index);
      cv::Mat tmp_image = (it->second).clone();
      putText(tmp_image, "index:  " + to_string(tmp_index) + "loop score:" + to_string(ret[i].Score), cv::Point2f(10, 50), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255));
      cv::hconcat(loop_result, tmp_image, loop_result);
    }
  }
  // a good match with its nerghbour
  if (ret.size() >= 1 && ret[0].Score > 0.05)  //最好的一帧相似性达到0.05进一步判断
                                               // TODO sore
                                               // if (ret.size() >= 1 && ret[0].Score > 0.020)  //最好的一帧相似性达到0.05进一步判断
    for (unsigned int i = 1; i < ret.size(); i++) {
      // if (ret[i].Score > ret[0].Score * 0.3)
      if (ret[i].Score > 0.015)  //书上介绍最好不要用固定的数值
      // if (ret[i].Score > 0.005)  //书上介绍最好不要用固定的数值
      {
        find_loop = true;
        int tmp_index = ret[i].Id;
        if (DEBUG_IMAGE && 0) {
          auto it = image_pool.find(tmp_index);
          cv::Mat tmp_image = (it->second).clone();
          cv::putText(tmp_image, "loop score:" + to_string(ret[i].Score), cv::Point2f(10, 50), CV_FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255));
          cv::hconcat(loop_result, tmp_image, loop_result);
        }
      }
    }
  /*
      if (DEBUG_IMAGE)
      {
          cv::imshow("loop_result", loop_result);
          cv::waitKey(20);
      }
  */
  if (find_loop && frame_index > 50)  //跳过最开始的50帧
  {
    int min_index = -1;
    for (unsigned int i = 0; i < ret.size(); i++)  // 4帧最相近的关键帧
    {
      // if (min_index == -1 || (ret[i].Id < min_index && ret[i].Score > 0.015)) min_index = ret[i].Id;
      if (min_index == -1 || (ret[i].Id < min_index && ret[i].Score > 0.005)) min_index = ret[i].Id;
    }
    // std::cout << "min_index: " << min_index << std::endl;
    return min_index;  //返回最匹配的帧id,且该帧是比较旧的
  } else
    return -1;
}

void PoseGraph::addKeyFrameIntoVoc(KeyFrame* keyframe) {
  // put image into image_pool; for visualization
  cv::Mat compressed_image;
  if (DEBUG_IMAGE) {
    int feature_num = keyframe->keypoints.size();
    cv::resize(keyframe->image, compressed_image, cv::Size(376, 240));
    putText(compressed_image, "feature_num:" + to_string(feature_num), cv::Point2f(10, 10), CV_FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255));
    image_pool[keyframe->index] = compressed_image;
  }
  db.add(keyframe->brief_descriptors);
}

void PoseGraph::optimize4DoF() {
  while (true) {
    int cur_index = -1;
    int first_looped_index = -1;
    m_optimize_buf.lock();
    while (!optimize_buf.empty()) {
      cur_index = optimize_buf.front();  //只要最新的回环信息,意味着处理不过来会放弃部分回环
      first_looped_index = earliest_loop_index;
      optimize_buf.pop();
    }
    m_optimize_buf.unlock();
    if (cur_index != -1) {
      printf("optimize pose graph \n");
      TicToc tmp_t;
      m_keyframelist.lock();                      //不允许插入新帧
      KeyFrame* cur_kf = getKeyFrame(cur_index);  //获取新帧

      int max_length = cur_index + 1;  //总帧数长度

      // w^t_i   w^q_i
      double t_array[max_length][3];
      Quaterniond q_array[max_length];
      double euler_array[max_length][3];
      double sequence_array[max_length];

      ceres::Problem problem;
      ceres::Solver::Options options;
      options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
      // options.minimizer_progress_to_stdout = true;
      // options.max_solver_time_in_seconds = SOLVER_TIME * 3;
      options.max_num_iterations = 5;
      ceres::Solver::Summary summary;
      ceres::LossFunction* loss_function;
      loss_function = new ceres::HuberLoss(0.1);
      // loss_function = new ceres::CauchyLoss(1.0);
      ceres::LocalParameterization* angle_local_parameterization = AngleLocalParameterization::Create();

      list<KeyFrame*>::iterator it;

      int i = 0;
      for (it = keyframelist.begin(); it != keyframelist.end(); it++) {
        if ((*it)->index < first_looped_index) continue;  //上次回环过的位置跳过
        (*it)->local_index = i;                           //回环后的id,当前循环下的id
        Quaterniond tmp_q;
        Matrix3d tmp_r;
        Vector3d tmp_t;
        (*it)->getVioPose(tmp_t, tmp_r);  //获取位姿
        tmp_q = tmp_r;
        t_array[i][0] = tmp_t(0);  //保存为数组形式,为ceres 做准备
        t_array[i][1] = tmp_t(1);
        t_array[i][2] = tmp_t(2);
        q_array[i] = tmp_q;

        Vector3d euler_angle = Utility::R2ypr(tmp_q.toRotationMatrix());  //将角度转换为欧拉角
        euler_array[i][0] = euler_angle.x();
        euler_array[i][1] = euler_angle.y();
        euler_array[i][2] = euler_angle.z();

        sequence_array[i] = (*it)->sequence;  //序列也保存为数组形式

        problem.AddParameterBlock(euler_array[i], 1, angle_local_parameterization);  //声明待优化变量类型 欧拉
        problem.AddParameterBlock(t_array[i], 3);

        //固定住首帧回环后的位姿   sequence没按'n' sequence=1,什么时候为0????
        if ((*it)->index == first_looped_index || (*it)->sequence == 0) {
          problem.SetParameterBlockConstant(euler_array[i]);
          problem.SetParameterBlockConstant(t_array[i]);
        }

        // add edge //添加位姿约束
        for (int j = 1; j < 5; j++) {  //除了添加本帧约束还有该帧与前面4帧之间的约束
          if (i - j >= 0 && sequence_array[i] == sequence_array[i - j]) {
            Vector3d euler_conncected = Utility::R2ypr(q_array[i - j].toRotationMatrix());  //前面帧的欧拉角
            //添加的约束为帧之间的相对位姿  x y z
            Vector3d relative_t(t_array[i][0] - t_array[i - j][0], t_array[i][1] - t_array[i - j][1], t_array[i][2] - t_array[i - j][2]);
            relative_t = q_array[i - j].inverse() * relative_t;
            //相对yaw角,只优化yaw角减少约束量??
            double relative_yaw = euler_array[i][0] - euler_array[i - j][0];
            ceres::CostFunction* cost_function = FourDOFError::Create(relative_t.x(), relative_t.y(), relative_t.z(), relative_yaw, euler_conncected.y(), euler_conncected.z());
            //优化变量为前面帧i-j位姿,该帧i位姿
            problem.AddResidualBlock(cost_function, NULL, euler_array[i - j], t_array[i - j], euler_array[i], t_array[i]);
          }
        }

        // add loop edge 回环约束
        if ((*it)->has_loop) {
          assert((*it)->loop_index >= first_looped_index);                    //要回环的id 做什么???

          int connected_index = getKeyFrame((*it)->loop_index)->local_index;  //若是该帧产生了回环,对应的回环帧在前面已经添加了local_index
          Vector3d euler_conncected = Utility::R2ypr(q_array[connected_index].toRotationMatrix());
          // printf("connected_index %d\n", connected_index);
          // printf("loop_index %d\n", (*it)->loop_index);
          Vector3d relative_t;
          relative_t = (*it)->getLoopRelativeT();
          double relative_yaw = (*it)->getLoopRelativeYaw();
          // std::cout << "relative_yaw: " << relative_yaw << std::endl;
          // std::cout << "relative_t: " << relative_t.transpose() << std::endl;
          ceres::CostFunction* cost_function = FourDOFWeightError::Create(relative_t.x(), relative_t.y(), relative_t.z(), relative_yaw, euler_conncected.y(), euler_conncected.z());
          problem.AddResidualBlock(cost_function, loss_function, euler_array[connected_index], t_array[connected_index], euler_array[i], t_array[i]);
        }

        if ((*it)->index == cur_index) break;
        i++;
      }
      m_keyframelist.unlock();

      ceres::Solve(options, &problem, &summary);
      // std::cout << summary.BriefReport() << "\n";

      // printf("pose optimization time: %f \n", tmp_t.toc());
      /*
      for (int j = 0 ; j < i; j++)
      {
          printf("optimize i: %d p: %f, %f, %f\n", j, t_array[j][0], t_array[j][1], t_array[j][2] );
      }
      */
      m_keyframelist.lock();
      i = 0;
      for (it = keyframelist.begin(); it != keyframelist.end(); it++) {
        if ((*it)->index < first_looped_index) continue;
        Quaterniond tmp_q;
        tmp_q = Utility::ypr2R(Vector3d(euler_array[i][0], euler_array[i][1], euler_array[i][2]));
        Vector3d tmp_t = Vector3d(t_array[i][0], t_array[i][1], t_array[i][2]);
        Matrix3d tmp_r = tmp_q.toRotationMatrix();
        (*it)->updatePose(tmp_t, tmp_r);

        if ((*it)->index == cur_index) break;
        i++;
      }

      Vector3d cur_t, vio_t;
      Matrix3d cur_r, vio_r;
      cur_kf->getPose(cur_t, cur_r);
      cur_kf->getVioPose(vio_t, vio_r);
      m_drift.lock();
      yaw_drift = Utility::R2ypr(cur_r).x() - Utility::R2ypr(vio_r).x();
      r_drift = Utility::ypr2R(Vector3d(yaw_drift, 0, 0));
      t_drift = cur_t - r_drift * vio_t;
      m_drift.unlock();
      // cout << "t_drift " << t_drift.transpose() << endl;
      // cout << "r_drift " << Utility::R2ypr(r_drift).transpose() << endl;
      // cout << "yaw drift " << yaw_drift << endl;

      it++;
      for (; it != keyframelist.end(); it++) {
        Vector3d P;
        Matrix3d R;
        (*it)->getVioPose(P, R);
        P = r_drift * P + t_drift;
        R = r_drift * R;
        (*it)->updatePose(P, R);
      }
      m_keyframelist.unlock();
      updatePath();
    }

    std::chrono::milliseconds dura(2000);
    std::this_thread::sleep_for(dura);
  }
  return;
}

void PoseGraph::optimize6DoF() {
  while (true) {
    int cur_index = -1;
    int first_looped_index = -1;
    m_optimize_buf.lock();
    while (!optimize_buf.empty()) {
      cur_index = optimize_buf.front();
      first_looped_index = earliest_loop_index;
      optimize_buf.pop();
    }
    m_optimize_buf.unlock();
    if (cur_index != -1) {
      printf("optimize pose graph \n");
      TicToc tmp_t;
      m_keyframelist.lock();
      KeyFrame* cur_kf = getKeyFrame(cur_index);

      int max_length = cur_index + 1;

      // w^t_i   w^q_i
      double t_array[max_length][3];
      double q_array[max_length][4];
      double sequence_array[max_length];

      ceres::Problem problem;
      ceres::Solver::Options options;
      options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
      // ptions.minimizer_progress_to_stdout = true;
      // options.max_solver_time_in_seconds = SOLVER_TIME * 3;
      options.max_num_iterations = 5;
      ceres::Solver::Summary summary;
      ceres::LossFunction* loss_function;
      loss_function = new ceres::HuberLoss(0.1);
      // loss_function = new ceres::CauchyLoss(1.0);
      ceres::LocalParameterization* local_parameterization = new ceres::QuaternionParameterization();

      list<KeyFrame*>::iterator it;

      int i = 0;
      for (it = keyframelist.begin(); it != keyframelist.end(); it++) {
        if ((*it)->index < first_looped_index) continue;
        (*it)->local_index = i;
        Quaterniond tmp_q;
        Matrix3d tmp_r;
        Vector3d tmp_t;
        (*it)->getVioPose(tmp_t, tmp_r);
        tmp_q = tmp_r;
        t_array[i][0] = tmp_t(0);
        t_array[i][1] = tmp_t(1);
        t_array[i][2] = tmp_t(2);
        q_array[i][0] = tmp_q.w();
        q_array[i][1] = tmp_q.x();
        q_array[i][2] = tmp_q.y();
        q_array[i][3] = tmp_q.z();

        sequence_array[i] = (*it)->sequence;

        problem.AddParameterBlock(q_array[i], 4, local_parameterization);
        problem.AddParameterBlock(t_array[i], 3);

        if ((*it)->index == first_looped_index || (*it)->sequence == 0) {
          problem.SetParameterBlockConstant(q_array[i]);
          problem.SetParameterBlockConstant(t_array[i]);
        }

        // add edge
        for (int j = 1; j < 5; j++) {
          if (i - j >= 0 && sequence_array[i] == sequence_array[i - j]) {
            Vector3d relative_t(t_array[i][0] - t_array[i - j][0], t_array[i][1] - t_array[i - j][1], t_array[i][2] - t_array[i - j][2]);
            Quaterniond q_i_j = Quaterniond(q_array[i - j][0], q_array[i - j][1], q_array[i - j][2], q_array[i - j][3]);
            Quaterniond q_i = Quaterniond(q_array[i][0], q_array[i][1], q_array[i][2], q_array[i][3]);
            relative_t = q_i_j.inverse() * relative_t;
            Quaterniond relative_q = q_i_j.inverse() * q_i;
            ceres::CostFunction* vo_function = RelativeRTError::Create(relative_t.x(), relative_t.y(), relative_t.z(), relative_q.w(), relative_q.x(), relative_q.y(), relative_q.z(), 0.1, 0.01);
            problem.AddResidualBlock(vo_function, NULL, q_array[i - j], t_array[i - j], q_array[i], t_array[i]);
          }
        }

        // add loop edge

        if ((*it)->has_loop) {
          assert((*it)->loop_index >= first_looped_index);
          int connected_index = getKeyFrame((*it)->loop_index)->local_index;
          Vector3d relative_t;
          relative_t = (*it)->getLoopRelativeT();
          Quaterniond relative_q;
          relative_q = (*it)->getLoopRelativeQ();
          ceres::CostFunction* loop_function = RelativeRTError::Create(relative_t.x(), relative_t.y(), relative_t.z(), relative_q.w(), relative_q.x(), relative_q.y(), relative_q.z(), 0.1, 0.01);
          problem.AddResidualBlock(loop_function, loss_function, q_array[connected_index], t_array[connected_index], q_array[i], t_array[i]);
        }

        if ((*it)->index == cur_index) break;
        i++;
      }
      m_keyframelist.unlock();

      ceres::Solve(options, &problem, &summary);
      // std::cout << summary.BriefReport() << "\n";

      // printf("pose optimization time: %f \n", tmp_t.toc());
      /*
      for (int j = 0 ; j < i; j++)
      {
          printf("optimize i: %d p: %f, %f, %f\n", j, t_array[j][0], t_array[j][1], t_array[j][2] );
      }
      */
      m_keyframelist.lock();
      i = 0;
      for (it = keyframelist.begin(); it != keyframelist.end(); it++) {
        if ((*it)->index < first_looped_index) continue;
        Quaterniond tmp_q(q_array[i][0], q_array[i][1], q_array[i][2], q_array[i][3]);
        Vector3d tmp_t = Vector3d(t_array[i][0], t_array[i][1], t_array[i][2]);
        Matrix3d tmp_r = tmp_q.toRotationMatrix();
        (*it)->updatePose(tmp_t, tmp_r);

        if ((*it)->index == cur_index) break;
        i++;
      }

      Vector3d cur_t, vio_t;
      Matrix3d cur_r, vio_r;
      cur_kf->getPose(cur_t, cur_r);
      cur_kf->getVioPose(vio_t, vio_r);
      m_drift.lock();
      r_drift = cur_r * vio_r.transpose();
      t_drift = cur_t - r_drift * vio_t;
      m_drift.unlock();
      // cout << "t_drift " << t_drift.transpose() << endl;
      // cout << "r_drift " << Utility::R2ypr(r_drift).transpose() << endl;

      it++;
      for (; it != keyframelist.end(); it++) {
        Vector3d P;
        Matrix3d R;
        (*it)->getVioPose(P, R);
        P = r_drift * P + t_drift;
        R = r_drift * R;
        (*it)->updatePose(P, R);
      }
      m_keyframelist.unlock();
      updatePath();
    }

    std::chrono::milliseconds dura(2000);
    std::this_thread::sleep_for(dura);
  }
  return;
}

void PoseGraph::updatePath() {
  m_keyframelist.lock();
  list<KeyFrame*>::iterator it;
  for (int i = 1; i <= sequence_cnt; i++) {
    path[i].poses.clear();
  }
  base_path.poses.clear();
  posegraph_visualization->reset();

  if (SAVE_LOOP_PATH) {
    ofstream loop_path_file_tmp(VINS_RESULT_PATH, ios::out);
    loop_path_file_tmp.close();
  }

  for (it = keyframelist.begin(); it != keyframelist.end(); it++) {
    Vector3d P;
    Matrix3d R;
    (*it)->getPose(P, R);
    Quaterniond Q;
    Q = R;
    //        printf("path p: %f, %f, %f\n",  P.x(),  P.z(),  P.y() );

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time((*it)->time_stamp);
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose.position.x = P.x() + VISUALIZATION_SHIFT_X;
    pose_stamped.pose.position.y = P.y() + VISUALIZATION_SHIFT_Y;
    pose_stamped.pose.position.z = P.z();
    pose_stamped.pose.orientation.x = Q.x();
    pose_stamped.pose.orientation.y = Q.y();
    pose_stamped.pose.orientation.z = Q.z();
    pose_stamped.pose.orientation.w = Q.w();
    if ((*it)->sequence == 0) {
      base_path.poses.push_back(pose_stamped);
      base_path.header = pose_stamped.header;
    } else {
      path[(*it)->sequence].poses.push_back(pose_stamped);
      path[(*it)->sequence].header = pose_stamped.header;
    }

    if (SAVE_LOOP_PATH) {
      ofstream loop_path_file(VINS_RESULT_PATH, ios::app);
      loop_path_file.setf(ios::fixed, ios::floatfield);
      loop_path_file.precision(0);
      loop_path_file << (*it)->time_stamp * 1e9 << ",";
      loop_path_file.precision(5);
      loop_path_file << P.x() << "," << P.y() << "," << P.z() << "," << Q.w() << "," << Q.x() << "," << Q.y() << "," << Q.z() << "," << endl;
      loop_path_file.close();
    }
    // draw local connection
    if (SHOW_S_EDGE) {
      list<KeyFrame*>::reverse_iterator rit = keyframelist.rbegin();
      list<KeyFrame*>::reverse_iterator lrit;
      for (; rit != keyframelist.rend(); rit++) {
        if ((*rit)->index == (*it)->index) {
          lrit = rit;
          lrit++;
          for (int i = 0; i < 4; i++) {
            if (lrit == keyframelist.rend()) break;
            if ((*lrit)->sequence == (*it)->sequence) {
              Vector3d conncected_P;
              Matrix3d connected_R;
              (*lrit)->getPose(conncected_P, connected_R);
              posegraph_visualization->add_edge(P, conncected_P);
            }
            lrit++;
          }
          break;
        }
      }
    }
    if (SHOW_L_EDGE) {
      if ((*it)->has_loop && (*it)->sequence == sequence_cnt) {
        KeyFrame* connected_KF = getKeyFrame((*it)->loop_index);
        Vector3d connected_P;
        Matrix3d connected_R;
        connected_KF->getPose(connected_P, connected_R);
        //(*it)->getVioPose(P, R);
        (*it)->getPose(P, R);
        if ((*it)->sequence > 0) {
          posegraph_visualization->add_loopedge(P, connected_P + Vector3d(VISUALIZATION_SHIFT_X, VISUALIZATION_SHIFT_Y, 0));
        }
      }
    }
  }
  publish();
  m_keyframelist.unlock();
}
void PoseGraph::savePoseGraphProto() {
  m_keyframelist.lock();
  TicToc tmp_t;
  MapProto map_out;
  printf("pose graph protobuf saving... \n");
  list<KeyFrame*>::iterator it;
  for (it = keyframelist.begin(); it != keyframelist.end(); it++)  // frames
  {
    FrameProto* frame = map_out.add_frames();
    frame->set_frame_id((*it)->index);
    for (int i = 0; i < (int)(*it)->keypoints.size(); i++)  // keypoints
    {
      std::string s;
      boost::to_string((*it)->brief_descriptors[i], s);
      DescriptorsProto* des = frame->add_descriptors();
      des->set_bitset(s);

      ImageNormPointProto* image_norm_point = frame->add_image_norm_points();
      image_norm_point->set_u((*it)->keypoints_norm[i].pt.x);
      image_norm_point->set_v((*it)->keypoints_norm[i].pt.y);

      ImagePointProto* image_point = frame->add_image_points();
      image_point->set_u((*it)->keypoints[i].pt.x);
      image_point->set_v((*it)->keypoints[i].pt.y);
    }

    Quaterniond PG_tmp_Q{(*it)->R_w_i};
    Vector3d PG_tmp_T = (*it)->T_w_i;

    PoseProto* pose = frame->mutable_frame_pose();
    pose->set_x(PG_tmp_T.x());
    pose->set_y(PG_tmp_T.y());
    pose->set_z(PG_tmp_T.z());
    pose->set_q_x(PG_tmp_Q.x());
    pose->set_q_y(PG_tmp_Q.y());
    pose->set_q_z(PG_tmp_Q.z());
    pose->set_q_w(PG_tmp_Q.w());

    frame->set_loop_index((*it)->loop_index);
  }
  printf("pose graph path: %s\n", POSE_GRAPH_SAVE_PATH.c_str());
  string file_path = POSE_GRAPH_SAVE_PATH + "pose_graph.pb";
  std::fstream output(file_path, std::ios::out | std::ios::trunc | std::ios::binary);
  if (!map_out.SerializeToOstream(&output)) {
    std::cerr << "Failed to write msg." << std::endl;
  }
  output.close();
  google::protobuf::ShutdownProtobufLibrary();
  printf("save pose graph time: %f s\n", tmp_t.toc() / 1000);
  m_keyframelist.unlock();
}
void PoseGraph::loadPoseGraphProto() {
  TicToc tmp_t;
  int load_frame_cnt = 0; //统计加载地图的帧数,优化时不对这部分frmae list 优化//还是说也将地图一起优化了???
  string file_path = POSE_GRAPH_SAVE_PATH + "pose_graph.pb";
  printf("pose graph Protobuf loading...\n");

  MapProto map_in;
  std::fstream input(file_path, std::ios::in | std::ios::binary);
  if (!map_in.ParseFromIstream(&input)) {
    std::cerr << "Failed to parse map." << std::endl;
    input.close();
    return;
  }
  input.close();

  int index;
  double time_stamp = 1;
  double PG_Tx, PG_Ty, PG_Tz;
  double PG_Qw, PG_Qx, PG_Qy, PG_Qz;
  int loop_index;
  // int keypoints_num;

  std::cout << "frames_size: " << map_in.frames_size() << std::endl;
  printf("xxxxxxx\n");

  for (int i = 0; i < map_in.frames_size(); i++) {
    FrameProto frame = map_in.frames(i);
    loop_index = frame.loop_index();

    // if (loop_index != -1)
    //   if (earliest_loop_index > loop_index || earliest_loop_index == -1) {
    //     earliest_loop_index = loop_index;
    //   }
    loop_index=-1; //默认所有加载帧都没回环
    index = frame.frame_id();

    PG_Tx = frame.frame_pose().x();
    PG_Ty = frame.frame_pose().y();
    PG_Tz = frame.frame_pose().z();
    PG_Qw = frame.frame_pose().q_w();
    PG_Qx = frame.frame_pose().q_x();
    PG_Qy = frame.frame_pose().q_y();
    PG_Qz = frame.frame_pose().q_z();

    Vector3d PG_T(PG_Tx, PG_Ty, PG_Tz);
    Quaterniond PG_Q;
    PG_Q.w() = PG_Qw;
    PG_Q.x() = PG_Qx;
    PG_Q.y() = PG_Qy;
    PG_Q.z() = PG_Qz;
    Matrix3d VIO_R, PG_R;
    PG_R = PG_Q.toRotationMatrix();

    vector<cv::KeyPoint> keypoints;
    vector<cv::KeyPoint> keypoints_norm;
    vector<BRIEF::bitset> brief_descriptors;
    for (int j = 0; j < frame.descriptors_size(); j++) {
      BRIEF::bitset tmp_des;
      DescriptorsProto des = frame.descriptors(j);
      string des_s = des.bitset();
      tmp_des = boost::dynamic_bitset<>(des_s);
      brief_descriptors.push_back(tmp_des);

      ImageNormPointProto norm_point = frame.image_norm_points(j);
      cv::KeyPoint tmp_keypoint_norm;
      tmp_keypoint_norm.pt.x = norm_point.u();
      tmp_keypoint_norm.pt.y = norm_point.v();
      keypoints_norm.push_back(tmp_keypoint_norm);

      ImagePointProto point = frame.image_points(j);
      cv::KeyPoint tmp_keypoint;
      tmp_keypoint.pt.x = point.u();
      tmp_keypoint.pt.y = point.v();
      keypoints.push_back(tmp_keypoint);
    }

    //并不需要 vio
    KeyFrame* keyframe = new KeyFrame(index, PG_T, PG_R, loop_index, keypoints, keypoints_norm, brief_descriptors);
    loadKeyFrame(keyframe, 0);  //不进行回环检测
    if (load_frame_cnt % 2 == 0) {
      publish();
    }
    load_frame_cnt++;
  }
}
void PoseGraph::savePoseGraph() {
  m_keyframelist.lock();
  TicToc tmp_t;
  FILE* pFile;
  printf("pose graph path: %s\n", POSE_GRAPH_SAVE_PATH.c_str());
  printf("pose graph saving... \n");
  string file_path = POSE_GRAPH_SAVE_PATH + "pose_graph.txt";
  pFile = fopen(file_path.c_str(), "w");
  // fprintf(pFile, "index time_stamp Tx Ty Tz Qw Qx Qy Qz loop_index loop_info\n");
  list<KeyFrame*>::iterator it;
  for (it = keyframelist.begin(); it != keyframelist.end(); it++) {
    std::string image_path, descriptor_path, brief_path, keypoints_path;
    if (DEBUG_IMAGE) {
      image_path = POSE_GRAPH_SAVE_PATH + to_string((*it)->index) + "_image.png";
      imwrite(image_path.c_str(), (*it)->image);
    }
    Quaterniond VIO_tmp_Q{(*it)->vio_R_w_i};
    Quaterniond PG_tmp_Q{(*it)->R_w_i};
    Vector3d VIO_tmp_T = (*it)->vio_T_w_i;
    Vector3d PG_tmp_T = (*it)->T_w_i;

    fprintf(pFile, " %d %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %d %f %f %f %f %f %f %f %f %d\n", (*it)->index, (*it)->time_stamp, VIO_tmp_T.x(), VIO_tmp_T.y(), VIO_tmp_T.z(), PG_tmp_T.x(), PG_tmp_T.y(), PG_tmp_T.z(), VIO_tmp_Q.w(), VIO_tmp_Q.x(), VIO_tmp_Q.y(), VIO_tmp_Q.z(), PG_tmp_Q.w(), PG_tmp_Q.x(), PG_tmp_Q.y(), PG_tmp_Q.z(), (*it)->loop_index, (*it)->loop_info(0), (*it)->loop_info(1), (*it)->loop_info(2), (*it)->loop_info(3), (*it)->loop_info(4), (*it)->loop_info(5), (*it)->loop_info(6), (*it)->loop_info(7), (int)(*it)->keypoints.size());

    // write keypoints, brief_descriptors   vector<cv::KeyPoint> keypoints vector<BRIEF::bitset> brief_descriptors;
    assert((*it)->keypoints.size() == (*it)->brief_descriptors.size());
    brief_path = POSE_GRAPH_SAVE_PATH + to_string((*it)->index) + "_briefdes.dat";
    std::ofstream brief_file(brief_path, std::ios::binary);
    keypoints_path = POSE_GRAPH_SAVE_PATH + to_string((*it)->index) + "_keypoints.txt";
    FILE* keypoints_file;
    keypoints_file = fopen(keypoints_path.c_str(), "w");
    for (int i = 0; i < (int)(*it)->keypoints.size(); i++) {
      brief_file << (*it)->brief_descriptors[i] << endl;
      fprintf(keypoints_file, "%f %f %f %f\n", (*it)->keypoints[i].pt.x, (*it)->keypoints[i].pt.y, (*it)->keypoints_norm[i].pt.x, (*it)->keypoints_norm[i].pt.y);
    }
    brief_file.close();
    fclose(keypoints_file);
  }
  fclose(pFile);

  printf("save pose graph time: %f s\n", tmp_t.toc() / 1000);
  m_keyframelist.unlock();
}
void PoseGraph::loadPoseGraph() {
  TicToc tmp_t;
  FILE* pFile;
  string file_path = POSE_GRAPH_SAVE_PATH + "pose_graph.txt";
  printf("lode pose graph from: %s \n", file_path.c_str());
  printf("pose graph loading...\n");
  pFile = fopen(file_path.c_str(), "r");
  if (pFile == NULL) {
    printf("lode previous pose graph error: wrong previous pose graph path or no previous pose graph \n the system will start with new pose graph \n");
    return;
  }
  int index;
  double time_stamp;
  double VIO_Tx, VIO_Ty, VIO_Tz;
  double PG_Tx, PG_Ty, PG_Tz;
  double VIO_Qw, VIO_Qx, VIO_Qy, VIO_Qz;
  double PG_Qw, PG_Qx, PG_Qy, PG_Qz;
  double loop_info_0, loop_info_1, loop_info_2, loop_info_3;
  double loop_info_4, loop_info_5, loop_info_6, loop_info_7;
  int loop_index;
  int keypoints_num;
  Eigen::Matrix<double, 8, 1> loop_info;
  int cnt = 0;
  while (fscanf(pFile, "%d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d %lf %lf %lf %lf %lf %lf %lf %lf %d", &index, &time_stamp, &VIO_Tx, &VIO_Ty, &VIO_Tz, &PG_Tx, &PG_Ty, &PG_Tz, &VIO_Qw, &VIO_Qx, &VIO_Qy, &VIO_Qz, &PG_Qw, &PG_Qx, &PG_Qy, &PG_Qz, &loop_index, &loop_info_0, &loop_info_1, &loop_info_2, &loop_info_3, &loop_info_4, &loop_info_5, &loop_info_6, &loop_info_7, &keypoints_num) != EOF) {
    /*
    printf("I read: %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d %lf %lf %lf %lf %lf %lf %lf %lf %d\n", index, time_stamp,
                                VIO_Tx, VIO_Ty, VIO_Tz,
                                PG_Tx, PG_Ty, PG_Tz,
                                VIO_Qw, VIO_Qx, VIO_Qy, VIO_Qz,
                                PG_Qw, PG_Qx, PG_Qy, PG_Qz,
                                loop_index,
                                loop_info_0, loop_info_1, loop_info_2, loop_info_3,
                                loop_info_4, loop_info_5, loop_info_6, loop_info_7,
                                keypoints_num);
    */
    cv::Mat image;
    std::string image_path, descriptor_path;
    if (DEBUG_IMAGE) {
      image_path = POSE_GRAPH_SAVE_PATH + to_string(index) + "_image.png";
      image = cv::imread(image_path.c_str(), 0);
    }

    Vector3d VIO_T(VIO_Tx, VIO_Ty, VIO_Tz);
    Vector3d PG_T(PG_Tx, PG_Ty, PG_Tz);
    Quaterniond VIO_Q;
    VIO_Q.w() = VIO_Qw;
    VIO_Q.x() = VIO_Qx;
    VIO_Q.y() = VIO_Qy;
    VIO_Q.z() = VIO_Qz;
    Quaterniond PG_Q;
    PG_Q.w() = PG_Qw;
    PG_Q.x() = PG_Qx;
    PG_Q.y() = PG_Qy;
    PG_Q.z() = PG_Qz;
    Matrix3d VIO_R, PG_R;
    VIO_R = VIO_Q.toRotationMatrix();
    PG_R = PG_Q.toRotationMatrix();
    Eigen::Matrix<double, 8, 1> loop_info;
    loop_info << loop_info_0, loop_info_1, loop_info_2, loop_info_3, loop_info_4, loop_info_5, loop_info_6, loop_info_7;

    if (loop_index != -1)
      if (earliest_loop_index > loop_index || earliest_loop_index == -1) {
        earliest_loop_index = loop_index;
      }

    // load keypoints, brief_descriptors
    string brief_path = POSE_GRAPH_SAVE_PATH + to_string(index) + "_briefdes.dat";
    std::ifstream brief_file(brief_path, std::ios::binary);
    string keypoints_path = POSE_GRAPH_SAVE_PATH + to_string(index) + "_keypoints.txt";
    FILE* keypoints_file;
    keypoints_file = fopen(keypoints_path.c_str(), "r");
    vector<cv::KeyPoint> keypoints;
    vector<cv::KeyPoint> keypoints_norm;
    vector<BRIEF::bitset> brief_descriptors;
    for (int i = 0; i < keypoints_num; i++) {
      BRIEF::bitset tmp_des;
      brief_file >> tmp_des;
      brief_descriptors.push_back(tmp_des);
      cv::KeyPoint tmp_keypoint;
      cv::KeyPoint tmp_keypoint_norm;
      double p_x, p_y, p_x_norm, p_y_norm;
      if (!fscanf(keypoints_file, "%lf %lf %lf %lf", &p_x, &p_y, &p_x_norm, &p_y_norm)) printf(" fail to load pose graph \n");
      tmp_keypoint.pt.x = p_x;
      tmp_keypoint.pt.y = p_y;
      tmp_keypoint_norm.pt.x = p_x_norm;
      tmp_keypoint_norm.pt.y = p_y_norm;
      keypoints.push_back(tmp_keypoint);
      keypoints_norm.push_back(tmp_keypoint_norm);
    }
    brief_file.close();
    fclose(keypoints_file);

    KeyFrame* keyframe = new KeyFrame(time_stamp, index, VIO_T, VIO_R, PG_T, PG_R, image, loop_index, loop_info, keypoints, keypoints_norm, brief_descriptors);
    loadKeyFrame(keyframe, 0);
    if (cnt % 20 == 0) {
      publish();
    }
    cnt++;
  }
  fclose(pFile);
  printf("load pose graph time: %f s\n", tmp_t.toc() / 1000);
  base_sequence = 0;
}

void PoseGraph::publish() {
  for (int i = 1; i <= sequence_cnt; i++) {
    // if (sequence_loop[i] == true || i == base_sequence)
    if (1 || i == base_sequence) {
      pub_pg_path.publish(path[i]);
      pub_path[i].publish(path[i]);
      posegraph_visualization->publish_by(pub_pose_graph, path[sequence_cnt].header);
    }
  }
  pub_base_path.publish(base_path);
  // posegraph_visualization->publish_by(pub_pose_graph, path[sequence_cnt].header);
}
