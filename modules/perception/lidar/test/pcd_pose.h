/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#ifndef PERCEPTION_LIDAR_TEST_PCD_POSE_H_
#define PERCEPTION_LIDAR_TEST_PCD_POSE_H_

#include <algorithm>
#include <fstream>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "modules/perception/base/point_cloud_types.h"
#include "modules/perception/lib/io/file_util.h"

namespace apollo {
namespace perception {
namespace lidar {

template <typename T>
void QuaternionToRotationMatrix(const T* quat, T* R) {
  T x2 = quat[0] * quat[0];
  T xy = quat[0] * quat[1];
  T rx = quat[3] * quat[0];
  T y2 = quat[1] * quat[1];
  T yz = quat[1] * quat[2];
  T ry = quat[3] * quat[1];
  T z2 = quat[2] * quat[2];
  T zx = quat[2] * quat[0];
  T rz = quat[3] * quat[2];
  T r2 = quat[3] * quat[3];
  R[0] = r2 + x2 - y2 - z2;  // fill diagonal terms
  R[4] = r2 - x2 + y2 - z2;
  R[8] = r2 - x2 - y2 + z2;
  R[3] = 2 * (xy + rz);  // fill off diagonal terms
  R[6] = 2 * (zx - ry);
  R[7] = 2 * (yz + rx);
  R[1] = 2 * (xy - rz);
  R[2] = 2 * (zx + ry);
  R[5] = 2 * (yz - rx);
}

int GetSeqNum(const std::string seq_str) {
  int start = seq_str.rfind("_");
  int end = seq_str.rfind(".");
  int seq_int = std::stoi(seq_str.substr(start + 1, end - start - 1));
  return seq_int;
}

bool FileCmp(const std::string& a, const std::string& b) {
  int int_a = GetSeqNum(a);
  int int_b = GetSeqNum(b);
  return int_a < int_b;
}

bool ReadPoseFile(const std::string& filename, Eigen::Matrix4d& pose,
                  int& frame_id, double& time_stamp) {
  std::ifstream ifs(filename.c_str());
  if (!ifs.is_open()) {
    std::cerr << "Failed to open file " << filename << std::endl;
    return false;
  }
  char buffer[1024];
  ifs.getline(buffer, 1024);
  int id = 0;
  double time_samp = 0;
  double quat[4];
  double matrix3x3[9];
  sscanf(buffer, "%d %lf %lf %lf %lf %lf %lf %lf %lf", &id, &(time_samp),
         &(pose(0, 3)), &(pose(1, 3)), &(pose(2, 3)), &(quat[0]), &(quat[1]),
         &(quat[2]), &(quat[3]));
  QuaternionToRotationMatrix<double>(quat, matrix3x3);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      pose(i, j) = matrix3x3[i * 3 + j];
    }
  }
  frame_id = id;
  time_stamp = time_samp;
  return true;
}

void LoadBPH(std::string pcd_file_name, base::PointFCloudPtr& pcd_ptr) {
  std::ifstream in_bph(pcd_file_name.c_str(), std::ios::binary);
  if (!in_bph.good()) {
    LOG(ERROR) << "input bph file is invalid";
    return;
  }

  in_bph.seekg(0, std::ios::end);
  unsigned int length = in_bph.tellg();
  in_bph.seekg(0, std::ios::beg);
  std::vector<float> data(length / sizeof(float));
  in_bph.read(reinterpret_cast<char*>(data.data()), length);
  in_bph.close();

  int org_cloud_size = data.size() / 4;
  pcd_ptr.reset(new base::PointFCloud);
  for (int np = 0; np < org_cloud_size; ++np) {
    const float& x = data.at(4 * np);
    const float& y = data.at(4 * np + 1);
    const float& z = data.at(4 * np + 2);
    const float& h = data.at(4 * np + 3);
    if (std::isnan(x) || std::isnan(y) || std::isnan(z) || std::isnan(h)) {
      continue;
    }

    base::PointF new_point;
    new_point.x = x;
    new_point.y = y;
    new_point.z = z;
    pcd_ptr->push_back(new_point);
    pcd_ptr->points_height(pcd_ptr->size() - 1) = h;
  }
}

void GetPcdPose(const std::string& pcd_dir, const std::string& pose_dir,
                std::vector<base::PointFCloudPtr>* pcds,
                std::vector<Eigen::Matrix4d>* poses) {
  std::string pcd_folder = pcd_dir;
  std::string pose_folder = pose_dir;
  std::vector<std::string> pcd_file_names;
  std::vector<std::string> pose_file_names;
  lib::FileUtil::GetFileList(pose_dir, ".pose", &pose_file_names);
  lib::FileUtil::GetFileList(pcd_dir, ".bph", &pcd_file_names);
  sort(pcd_file_names.begin(), pcd_file_names.end(), FileCmp);
  sort(pose_file_names.begin(), pose_file_names.end(), FileCmp);
  if (pose_file_names.size() != pcd_file_names.size()) {
    LOG_ERROR << "pcd file number does not match pose file number";
    LOG_ERROR << "pose_file_names : " << pose_file_names.size();
    LOG_ERROR << "pcd_file_names : " << pcd_file_names.size();
    return;
  }
  for (size_t i = 0; i < pcd_file_names.size(); ++i) {
    LOG_INFO << "pcd_file_names[" << i << "]: " << pcd_file_names[i];
    LOG_INFO << "pose_file_names[" << i << "]: " << pose_file_names[i];
    // read pcd
    base::PointFCloudPtr cloud_ptr(new base::PointFCloud);
    LoadBPH(pcd_file_names[i], cloud_ptr);
    pcds->push_back(cloud_ptr);
    // read pose
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    int frame_id = -1;
    double time_stamp = 0.0;
    if (!ReadPoseFile(pose_file_names[i], pose, frame_id, time_stamp)) {
      std::cout << "Failed to read file " << pose_file_names[i] << "\n";
    }
    poses->push_back(pose);
  }
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo

#endif  // PERCEPTION_LIDAR_TEST_PCD_POSE_H_
