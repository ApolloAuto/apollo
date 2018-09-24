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
#ifndef PERCEPTION_FUSION_BASE_SENSOR_DATA_MANAGER_H_
#define PERCEPTION_FUSION_BASE_SENSOR_DATA_MANAGER_H_
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "modules/perception/base/frame.h"
#include "modules/perception/common/sensor_manager/sensor_manager.h"
#include "modules/perception/fusion/base/sensor.h"
#include "modules/perception/fusion/base/sensor_frame.h"
#include "modules/perception/lib/singleton/singleton.h"

namespace apollo {
namespace perception {
namespace fusion {

class SensorDataManager {
 public:
  SensorDataManager(const SensorDataManager&) = delete;
  SensorDataManager operator=(const SensorDataManager&) = delete;

  bool Init();

  void Reset();

  void AddSensorMeasurements(const base::FrameConstPtr& frame_ptr);

  bool IsLidar(const base::FrameConstPtr& frame_ptr);
  bool IsRadar(const base::FrameConstPtr& frame_ptr);
  bool IsCamera(const base::FrameConstPtr& frame_ptr);

  // Getter
  void GetLatestSensorFrames(double timestamp, const std::string& sensor_id,
                             std::vector<SensorFramePtr>* frames) const;

  void GetLatestFrames(double timestamp,
                       std::vector<SensorFramePtr>* frames) const;

  bool GetPose(const std::string& sensor_id, double timestamp,
               Eigen::Affine3d* pose) const;

  base::BaseCameraModelPtr GetCameraIntrinsic(
      const std::string& sensor_id) const;

 private:
  SensorDataManager();
  ~SensorDataManager() = default;

  friend class lib::Singleton<SensorDataManager>;

 private:
  bool inited_ = false;
  std::unordered_map<std::string, SensorPtr> sensors_;

  const common::SensorManager* sensor_manager_ = nullptr;
};

}  // namespace fusion
}  // namespace perception
}  // namespace apollo

#endif  // PERCEPTION_FUSION_BASE_SENSOR_DATA_MANAGER_H_
