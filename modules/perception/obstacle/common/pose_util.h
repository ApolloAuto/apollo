/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#ifndef MODULES_PERCEPTION_OBSTACLE_COMMON_POSE_UTIL_H_
#define MODULES_PERCEPTION_OBSTACLE_COMMON_POSE_UTIL_H_

#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "Eigen/Dense"

namespace apollo {
namespace perception {

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

bool ReadPoseFile(const std::string& filename, Eigen::Matrix4d* pose,
                  int* frame_id, double* time_stamp);

bool ReadPoseFileMat12(const std::string &filename, Eigen::Matrix4d *pose,
                  int *frame_id, double *time_stamp);

// @brief: Load the velodyne extrinsic from a YAML file.
// @param [in]: path to yaml file
// @param [out]: extrinsic transform
bool LoadExtrinsic(const std::string& file_path, Eigen::Affine3d* extrinsic);

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_COMMON_POSE_UTIL_H_
