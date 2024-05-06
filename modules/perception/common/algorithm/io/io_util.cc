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
#include "modules/perception/common/algorithm/io/io_util.h"

#include <boost/filesystem.hpp>

#include "absl/strings/match.h"
#include "yaml-cpp/yaml.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/perception/common/base/camera.h"

namespace apollo {
namespace perception {
namespace algorithm {

using cyber::common::PathExists;

bool ReadPoseFile(const std::string &filename, Eigen::Affine3d *pose,
                  int *frame_id, double *time_stamp) {
  if (pose == nullptr || frame_id == nullptr || time_stamp == nullptr) {
    AERROR << "Nullptr error.";
    return false;
  }

  std::ifstream fin(filename.c_str());
  if (!fin.is_open()) {
    AERROR << "Failed to open pose file: " << filename;
    return false;
  }

  Eigen::Vector3d translation;
  Eigen::Quaterniond quat;
  fin >> *frame_id >> *time_stamp >> translation(0) >> translation(1) >>
      translation(2) >> quat.x() >> quat.y() >> quat.z() >> quat.w();

  *pose = Eigen::Affine3d::Identity();
  pose->prerotate(quat);
  pose->pretranslate(translation);

  fin.close();
  return true;
}

bool LoadBrownCameraIntrinsic(const std::string &yaml_file,
                              base::BrownCameraDistortionModel *model) {
  if (!PathExists(yaml_file) || model == nullptr) {
    return false;
  }

  YAML::Node node = YAML::LoadFile(yaml_file);
  if (node.IsNull()) {
    AINFO << "Load " << yaml_file << " failed! please check!";
    return false;
  }

  float camera_width = 0.0f;
  float camera_height = 0.0f;
  Eigen::VectorXf params(9 + 5 + 3);  // add k4, k5, k6
  try {
    camera_width = node["width"].as<float>();
    camera_height = node["height"].as<float>();
    for (size_t i = 0; i < 9; ++i) {
      params(i) = node["K"][i].as<float>();
    }
    for (size_t i = 0; i < 8; ++i) {
      params(9 + i) = node["D"][i].as<float>();
    }

    model->set_params(static_cast<size_t>(camera_width),
                      static_cast<size_t>(camera_height), params);
  } catch (YAML::Exception &e) {
    AERROR << "load camera intrisic file " << yaml_file
           << " with error, YAML exception: " << e.what();
    return false;
  }

  return true;
}

bool LoadOmnidirectionalCameraIntrinsics(
    const std::string &yaml_file,
    base::OmnidirectionalCameraDistortionModel *model) {
  if (!PathExists(yaml_file) || model == nullptr) {
    return false;
  }

  YAML::Node node = YAML::LoadFile(yaml_file);
  if (node.IsNull()) {
    AINFO << "Load " << yaml_file << " failed! please check!";
    return false;
  }

  if (!node["width"].IsDefined() || !node["height"].IsDefined() ||
      !node["center"].IsDefined() || !node["affine"].IsDefined() ||
      !node["cam2world"].IsDefined() || !node["world2cam"].IsDefined() ||
      !node["focallength"].IsDefined() || !node["principalpoint"].IsDefined()) {
    AINFO << "Invalid intrinsics file for an omnidirectional camera.";
    return false;
  }

  try {
    int camera_width = 0;
    int camera_height = 0;

    std::vector<float> params;  // center|affine|f|p|i,cam2world|j,world2cam

    camera_width = node["width"].as<int>();
    camera_height = node["height"].as<int>();

    params.push_back(node["center"]["x"].as<float>());
    params.push_back(node["center"]["y"].as<float>());

    params.push_back(node["affine"]["c"].as<float>());
    params.push_back(node["affine"]["d"].as<float>());
    params.push_back(node["affine"]["e"].as<float>());

    params.push_back(node["focallength"].as<float>());
    params.push_back(node["principalpoint"]["x"].as<float>());
    params.push_back(node["principalpoint"]["y"].as<float>());

    params.push_back(static_cast<float>(node["cam2world"].size()));

    for (size_t i = 0; i < node["cam2world"].size(); ++i) {
      params.push_back(node["cam2world"][i].as<float>());
    }

    params.push_back(static_cast<float>(node["world2cam"].size()));

    for (size_t i = 0; i < node["world2cam"].size(); ++i) {
      params.push_back(node["world2cam"][i].as<float>());
    }

    Eigen::VectorXf eigen_params(params.size());
    for (size_t i = 0; i < params.size(); ++i) {
      eigen_params(i) = params[i];
    }

    model->set_params(camera_width, camera_height, eigen_params);
  } catch (YAML::Exception &e) {
    AERROR << "load camera intrisic file " << yaml_file
           << " with error, YAML exception: " << e.what();
    return false;
  }

  return true;
}

bool GetFileList(const std::string &path, const std::string &suffix,
                 std::vector<std::string> *files) {
  if (!PathExists(path)) {
    AINFO << path << " not exist.";
    return false;
  }

  boost::filesystem::recursive_directory_iterator itr(path);
  while (itr != boost::filesystem::recursive_directory_iterator()) {
    try {
      if (absl::EndsWith(itr->path().string(), suffix)) {
        files->push_back(itr->path().string());
      }
      ++itr;
    } catch (const std::exception &ex) {
      AWARN << "Caught execption: " << ex.what();
      continue;
    }
  }
  return true;
}

}  // namespace algorithm
}  // namespace perception
}  // namespace apollo
