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
#ifndef PERCEPTION_FUSION_BASE_SENSOR_FRAME_H_
#define PERCEPTION_FUSION_BASE_SENSOR_FRAME_H_
#include <memory>
#include <string>
#include <vector>
#include "Eigen/Core"

#include "modules/perception/base/frame.h"
#include "modules/perception/fusion/base/base_forward_declaration.h"
#include "modules/perception/fusion/base/sensor_object.h"

namespace apollo {
namespace perception {
namespace fusion {

class SensorFrame : public std::enable_shared_from_this<SensorFrame> {
 public:
  SensorFrame() = default;

  // Unable to be called in constructor due to weak_ptr initialization
  void Initialize(const base::FrameConstPtr& base_frame_ptr,
                  const SensorPtr& sensor_ptr);

  // Getter
  inline double GetTimestamp() const { return timestamp_; }

  inline bool GetPose(Eigen::Affine3d* pose) const {
    assert(pose != nullptr);
    *pose = sensor2world_pose_;
    return true;
  }

  inline std::vector<SensorObjectPtr>& GetForegroundObjects() {
    return foreground_objects_;
  }

  inline const std::vector<SensorObjectPtr>& GetForegroundObjects() const {
    return foreground_objects_;
  }

  inline std::vector<SensorObjectPtr>& GetBackgroundObjects() {
    return background_objects_;
  }

  inline const std::vector<SensorObjectPtr>& GetBackgroundObjects() const {
    return background_objects_;
  }

  std::string GetSensorId() const;

  base::SensorType GetSensorType() const;

 private:
  inline SensorFramePtr GetPtr() { return shared_from_this(); }

  inline bool CheckSensorExist() const {
    bool expired = sensor_ptr_.expired();
    return !expired;
  }

 private:
  double timestamp_ = 0.0;
  Eigen::Affine3d sensor2world_pose_;

  std::vector<SensorObjectPtr> foreground_objects_;
  std::vector<SensorObjectPtr> background_objects_;

  // sensor-specific frame supplements
  base::LidarFrameSupplement lidar_frame_supplement_;
  base::RadarFrameSupplement radar_frame_supplement_;
  base::CameraFrameSupplement camera_frame_supplement_;

  std::weak_ptr<const Sensor> sensor_ptr_;
};

}  // namespace fusion
}  // namespace perception
}  // namespace apollo

#endif  // PERCEPTION_FUSION_BASE_SENSOR_FRAME_H_
