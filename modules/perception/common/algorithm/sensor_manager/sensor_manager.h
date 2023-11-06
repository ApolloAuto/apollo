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
#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include "cyber/common/macros.h"
#include "modules/perception/common/base/camera.h"
#include "modules/perception/common/base/distortion_model.h"
#include "modules/perception/common/base/sensor_meta.h"
#include "modules/perception/common/perception_gflags.h"

namespace apollo {
namespace perception {
namespace algorithm {

using apollo::perception::base::BaseCameraDistortionModel;
using apollo::perception::base::BaseCameraModel;

class SensorManager {
 public:
  bool Init();

  bool IsSensorExist(const std::string& name) const;

  bool GetSensorInfo(const std::string& name,
                     apollo::perception::base::SensorInfo* sensor_info) const;

  std::shared_ptr<BaseCameraDistortionModel> GetDistortCameraModel(
      const std::string& name) const;

  std::shared_ptr<BaseCameraModel> GetUndistortCameraModel(
      const std::string& name) const;

  // sensor type functions
  bool IsHdLidar(const std::string& name) const;
  bool IsHdLidar(const apollo::perception::base::SensorType& type) const;
  bool IsLdLidar(const std::string& name) const;
  bool IsLdLidar(const apollo::perception::base::SensorType& type) const;

  bool IsLidar(const std::string& name) const;
  bool IsLidar(const apollo::perception::base::SensorType& type) const;
  bool IsRadar(const std::string& name) const;
  bool IsRadar(const apollo::perception::base::SensorType& type) const;
  bool IsCamera(const std::string& name) const;
  bool IsCamera(const apollo::perception::base::SensorType& type) const;
  bool IsUltrasonic(const std::string& name) const;
  bool IsUltrasonic(const apollo::perception::base::SensorType& type) const;

  bool IsMainSensor(const std::string& name) const;

  // sensor frame id function
  std::string GetFrameId(const std::string& name) const;

 private:
  inline std::string IntrinsicPath(const std::string& frame_id) {
    std::string intrinsics =
        FLAGS_obs_sensor_intrinsic_path + "/" + frame_id + "_intrinsics.yaml";
    return intrinsics;
  }

 private:
  std::mutex mutex_;
  bool inited_ = false;

  std::unordered_map<std::string, apollo::perception::base::SensorInfo>
      sensor_info_map_;
  std::unordered_map<std::string, std::shared_ptr<BaseCameraDistortionModel>>
      distort_model_map_;
  std::unordered_map<std::string, std::shared_ptr<BaseCameraModel>>
      undistort_model_map_;

  DECLARE_SINGLETON(SensorManager)
};

}  // namespace algorithm
}  // namespace perception
}  // namespace apollo
