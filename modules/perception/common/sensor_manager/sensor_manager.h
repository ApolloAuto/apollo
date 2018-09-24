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
#ifndef PERCEPTION_COMMON_SENSOR_MANAGER_SENSOR_MANAGER_H_
#define PERCEPTION_COMMON_SENSOR_MANAGER_SENSOR_MANAGER_H_

#include <google/protobuf/message.h>
#include <google/protobuf/text_format.h>

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "gflags/gflags.h"

#include "modules/perception/base/camera.h"
#include "modules/perception/base/distortion_model.h"
#include "modules/perception/base/sensor_meta.h"
#include "modules/perception/lib/singleton/singleton.h"
#include "modules/perception/lib/thread/mutex.h"

namespace apollo {
namespace perception {
namespace common {

DECLARE_string(obs_sensor_intrinsic_path);

class SensorManager {
 public:
  SensorManager(const SensorManager&) = delete;
  SensorManager operator=(const SensorManager&) = delete;

  bool Init();

  bool IsSensorExist(const std::string& name) const;

  bool GetSensorInfo(const std::string& name,
                     base::SensorInfo* sensor_info) const;

  base::BaseCameraDistortionModelPtr GetDistortCameraModel(
      const std::string& name) const;

  base::BaseCameraModelPtr GetUndistortCameraModel(
      const std::string& name) const;

  // sensor type functions
  bool IsHdLidar(const std::string& name) const;
  bool IsHdLidar(base::SensorType type) const;
  bool IsLdLidar(const std::string& name) const;
  bool IsLdLidar(base::SensorType type) const;

  bool IsLidar(const std::string& name) const;
  bool IsLidar(base::SensorType type) const;
  bool IsRadar(const std::string& name) const;
  bool IsRadar(base::SensorType type) const;
  bool IsCamera(const std::string& name) const;
  bool IsCamera(base::SensorType type) const;
  bool IsUltrasonic(const std::string& name) const;
  bool IsUltrasonic(base::SensorType type) const;

  // sensor frame id function
  std::string GetFrameId(const std::string& name) const;

 private:
  SensorManager();
  ~SensorManager() = default;

  friend class lib::Singleton<SensorManager>;

  inline std::string IntrinsicPath(const std::string& frame_id) {
    std::string intrinsics =
        FLAGS_obs_sensor_intrinsic_path + "/" + frame_id + "_intrinsics.yaml";
    return intrinsics;
  }

 private:
  std::mutex mutex_;
  bool inited_ = false;

  std::unordered_map<std::string, base::SensorInfo> sensor_info_map_;

  std::unordered_map<std::string, base::BaseCameraDistortionModelPtr>
      distort_model_map_;

  std::unordered_map<std::string, base::BaseCameraModelPtr>
      undistort_model_map_;
};

}  // namespace common
}  // namespace perception
}  // namespace apollo

#endif  // PERCEPTION_COMMON_SENSOR_MANAGER_SENSOR_MANAGER_H_
