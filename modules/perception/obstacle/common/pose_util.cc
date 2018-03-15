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
#include "modules/perception/obstacle/common/pose_util.h"

#include "yaml-cpp/yaml.h"

#include "modules/common/log.h"
#include "modules/common/util/util.h"

namespace apollo {
namespace perception {

using apollo::common::util::QuaternionToRotationMatrix;

bool ReadPoseFile(const std::string &filename, Eigen::Matrix4d *pose,
                  int *frame_id, double *time_stamp) {
  *pose = Eigen::Matrix4d::Identity();
  std::ifstream ifs(filename.c_str());
  if (!ifs.is_open()) {
    AERROR << "Failed to open file " << filename;
    return false;
  }
  char buffer[1024];
  ifs.getline(buffer, 1024);
  int id = 0;
  double time_samp = 0;
  double quat[4];
  double matrix3x3[9];
  double &pose03 = (*pose)(0, 3);
  double &pose13 = (*pose)(1, 3);
  double &pose23 = (*pose)(2, 3);
  int ret = sscanf(buffer, "%d %lf %lf %lf %lf %lf %lf %lf %lf", &id,
                   &(time_samp), &pose03, &pose13, &pose23, &(quat[0]),
                   &(quat[1]), &(quat[2]), &(quat[3]));
  if (ret != 9) {
    AERROR << "Failed to scan parameters.";
    return false;
  }
  QuaternionToRotationMatrix<double>(quat, matrix3x3);

  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      (*pose)(i, j) = matrix3x3[i * 3 + j];
    }
  }

  (*frame_id) = id;
  (*time_stamp) = time_samp;
  return true;
}

bool ReadPoseFileMat12(const std::string &filename, Eigen::Matrix4d *pose,
                       int *frame_id, double *time_stamp) {
  *pose = Eigen::Matrix4d::Identity();
  std::ifstream ifs(filename.c_str());
  if (!ifs.is_open()) {
    AERROR << "Failed to open file " << filename;
    return false;
  }
  ifs >> *frame_id >> *time_stamp;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 4; ++j) {
      ifs >> (*pose)(i, j);
    }
  }
  return true;
}

bool LoadExtrinsic(const std::string &file_path, Eigen::Affine3d *extrinsic) {
  try {
    YAML::Node config = YAML::LoadFile(file_path);
    if (!config["transform"]) {
      return false;
    }
    if (config["transform"]["translation"] && config["transform"]["rotation"]) {
      double tx = config["transform"]["translation"]["x"].as<double>();
      double ty = config["transform"]["translation"]["y"].as<double>();
      double tz = config["transform"]["translation"]["z"].as<double>();

      double qx = config["transform"]["rotation"]["x"].as<double>();
      double qy = config["transform"]["rotation"]["y"].as<double>();
      double qz = config["transform"]["rotation"]["z"].as<double>();
      double qw = config["transform"]["rotation"]["w"].as<double>();
      *extrinsic =
          Eigen::Translation3d(tx, ty, tz) * Eigen::Quaterniond(qw, qx, qy, qz);
    }
  } catch (const YAML::Exception &e) {
    AERROR << "load extrinsics: " << file_path
           << " failed! error: " << e.what();
    return false;
  }
  return true;
}

}  // namespace perception
}  // namespace apollo
