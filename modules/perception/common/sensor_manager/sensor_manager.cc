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
#include "modules/perception/common/sensor_manager/sensor_manager.h"

#include <assert.h>

#include <iostream>
#include <string>

#include "modules/perception/base/log.h"
#include "modules/perception/common/io/io_util.h"
#include "modules/perception/common/sensor_manager/proto/sensor_meta_schema.pb.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/lib/io/file_util.h"

namespace apollo {
namespace perception {
namespace common {

DEFINE_string(obs_sensor_meta_path,
              "./data/perception/modules/perception/common/sensor_meta.pt",
              "The SensorManager config file.");
DEFINE_string(obs_sensor_intrinsic_path, "/home/caros/cybertron/params",
              "The intrinsics/extrinsics dir.");

SensorManager::SensorManager() { CHECK_EQ(this->Init(), true); }

bool SensorManager::Init() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (inited_) {
    return true;
  }

  sensor_info_map_.clear();
  distort_model_map_.clear();
  undistort_model_map_.clear();

  lib::ConfigManager* config_manager =
      lib::Singleton<lib::ConfigManager>::get_instance();
  assert(config_manager != nullptr);
  std::string file_path = lib::FileUtil::GetAbsolutePath(
      config_manager->work_root(), FLAGS_obs_sensor_meta_path);

  std::string content;
  if (!lib::FileUtil::GetFileContent(file_path, &content)) {
    LOG_ERROR << "Failed to get SensorManager config path: "
              << FLAGS_obs_sensor_meta_path;
    return false;
  }

  MultiSensorMeta sensor_list_proto;
  if (!google::protobuf::TextFormat::ParseFromString(content,
                                                     &sensor_list_proto)) {
    LOG_ERROR << "Invalid MultiSensorMeta file!";
    return false;
  }

  auto AddSensorInfo = [this](const SensorMeta& sensor_meta_proto) {
    base::SensorInfo sensor_info;
    sensor_info.name = sensor_meta_proto.name();
    sensor_info.type = static_cast<base::SensorType>(sensor_meta_proto.type());
    sensor_info.orientation =
        static_cast<base::SensorOrientation>(sensor_meta_proto.orientation());
    sensor_info.frame_id = sensor_meta_proto.name();

    auto pair = sensor_info_map_.insert(
        make_pair(sensor_meta_proto.name(), sensor_info));
    if (!pair.second) {
      LOG_ERROR << "Duplicate sensor name error.";
      return false;
    }

    if (this->IsCamera(sensor_info.type)) {
      base::BrownCameraDistortionModelPtr distort_model(
          new base::BrownCameraDistortionModel());
      if (!LoadBrownCameraIntrinsic(IntrinsicPath(sensor_info.frame_id),
                                    distort_model.get())) {
        LOG_ERROR << "Failed to load camera intrinsic";
        return false;
      }

      distort_model_map_.insert(
          make_pair(sensor_meta_proto.name(),
                    std::dynamic_pointer_cast<base::BaseCameraDistortionModel>(
                        distort_model)));
      undistort_model_map_.insert(make_pair(sensor_meta_proto.name(),
                                            distort_model->get_camera_model()));
    }
    return true;
  };

  for (const SensorMeta& sensor_meta_proto : sensor_list_proto.sensor_meta()) {
    if (!AddSensorInfo(sensor_meta_proto)) {
      LOG_ERROR << "Failed to add sensor_info: " << sensor_meta_proto.name();
      return false;
    }
  }

  inited_ = true;
  LOG_INFO << "Init sensor_manager success.";
  return true;
}

bool SensorManager::IsSensorExist(const std::string& name) const {
  const auto& itr = sensor_info_map_.find(name);
  if (itr == sensor_info_map_.end()) {
    return false;
  } else {
    return true;
  }
}

bool SensorManager::GetSensorInfo(const std::string& name,
                                  base::SensorInfo* sensor_info) const {
  if (sensor_info == nullptr) {
    LOG_ERROR << "Nullptr error.";
    return false;
  }

  const auto& itr = sensor_info_map_.find(name);
  if (itr == sensor_info_map_.end()) {
    return false;
  } else {
    *sensor_info = itr->second;
  }
  return true;
}

base::BaseCameraDistortionModelPtr SensorManager::GetDistortCameraModel(
    const std::string& name) const {
  const auto& itr = distort_model_map_.find(name);
  if (itr == distort_model_map_.end()) {
    return nullptr;
  } else {
    return itr->second;
  }
}

base::BaseCameraModelPtr SensorManager::GetUndistortCameraModel(
    const std::string& name) const {
  const auto& itr = undistort_model_map_.find(name);
  if (itr == undistort_model_map_.end()) {
    return nullptr;
  } else {
    return itr->second;
  }
}

bool SensorManager::IsHdLidar(const std::string& name) const {
  const auto& itr = sensor_info_map_.find(name);
  if (itr == sensor_info_map_.end()) {
    return false;
  } else {
    base::SensorType type = itr->second.type;
    return this->IsHdLidar(type);
  }
}

bool SensorManager::IsHdLidar(base::SensorType type) const {
  return type == base::SensorType::VELODYNE_64 ||
         type == base::SensorType::VELODYNE_32 ||
         type == base::SensorType::VELODYNE_16;
}

bool SensorManager::IsLdLidar(const std::string& name) const {
  const auto& itr = sensor_info_map_.find(name);
  if (itr == sensor_info_map_.end()) {
    return false;
  } else {
    base::SensorType type = itr->second.type;
    return this->IsLdLidar(type);
  }
}

bool SensorManager::IsLdLidar(base::SensorType type) const {
  return type == base::SensorType::LDLIDAR_4 ||
         type == base::SensorType::LDLIDAR_1;
}

bool SensorManager::IsLidar(const std::string& name) const {
  const auto& itr = sensor_info_map_.find(name);
  if (itr == sensor_info_map_.end()) {
    return false;
  } else {
    base::SensorType type = itr->second.type;
    return this->IsLidar(type);
  }
}

bool SensorManager::IsLidar(base::SensorType type) const {
  return this->IsHdLidar(type) || this->IsLdLidar(type);
}

bool SensorManager::IsRadar(const std::string& name) const {
  const auto& itr = sensor_info_map_.find(name);
  if (itr == sensor_info_map_.end()) {
    return false;
  } else {
    base::SensorType type = itr->second.type;
    return this->IsRadar(type);
  }
}

bool SensorManager::IsRadar(base::SensorType type) const {
  return type == base::SensorType::SHORT_RANGE_RADAR ||
         type == base::SensorType::LONG_RANGE_RADAR;
}

bool SensorManager::IsCamera(const std::string& name) const {
  const auto& itr = sensor_info_map_.find(name);
  if (itr == sensor_info_map_.end()) {
    return false;
  } else {
    base::SensorType type = itr->second.type;
    return this->IsCamera(type);
  }
}

bool SensorManager::IsCamera(base::SensorType type) const {
  return type == base::SensorType::MONOCULAR_CAMERA ||
         type == base::SensorType::STEREO_CAMERA;
}

bool SensorManager::IsUltrasonic(const std::string& name) const {
  const auto& itr = sensor_info_map_.find(name);
  if (itr == sensor_info_map_.end()) {
    return false;
  } else {
    base::SensorType type = itr->second.type;
    return this->IsUltrasonic(type);
  }
}

bool SensorManager::IsUltrasonic(base::SensorType type) const {
  return type == base::SensorType::ULTRASONIC;
}

std::string SensorManager::GetFrameId(const std::string& name) const {
  const auto& itr = sensor_info_map_.find(name);
  if (itr == sensor_info_map_.end()) {
    return std::string("");
  } else {
    return itr->second.frame_id;
  }
}

}  // namespace common
}  // namespace perception
}  // namespace apollo
