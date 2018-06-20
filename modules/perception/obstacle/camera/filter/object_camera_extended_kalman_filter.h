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

#ifndef MODULES_PERCEPTION_OBSTACLE_CAMERA_FILTER_OBJECT_CAMERA_KALMAN_FILTER_H_
#define MODULES_PERCEPTION_OBSTACLE_CAMERA_FILTER_OBJECT_CAMERA_KALMAN_FILTER_H_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "modules/common/math/extended_kalman_filter.h"
#include "modules/perception/obstacle/camera/common/visual_object.h"
#include "modules/perception/obstacle/camera/interface/base_camera_filter.h"

namespace apollo {
namespace perception {

class ObjectCameraExtendedKalmanFilter : public BaseCameraFilter {
 public:
  ObjectCameraExtendedKalmanFilter() : BaseCameraFilter() {}

  virtual ~ObjectCameraExtendedKalmanFilter() {}

  bool Init() override { return true; }

  bool Filter(const double timestamp,
              std::vector<std::shared_ptr<VisualObject>> *objects) override;

  std::string Name() const override;

  class ObjectFilter {
   public:
    ObjectFilter() {}

    ObjectFilter(const int track_id, const float last_timestamp) :
      track_id_(track_id), last_timestamp_(last_timestamp) {}

    int track_id_ = -1;
    int lost_frame_cnt_ = 0;
    float last_timestamp_ = 0.0f;
    common::math::ExtendedKalmanFilter<float, 4, 3, 1> object_config_filter_;
  };

 private:
  const int kMaxKeptFrameCnt = 5;

  void GetState(const int track_id,
      const std::shared_ptr<VisualObject>& obj_ptr);

  // @brief Predict step
  void Predict(const int track_id, const double timestamp);

  // @brief Update step
  void Update(const int track_id, const std::shared_ptr<VisualObject> &obj_ptr);

  ObjectFilter CreateObjectFilter(const int track_id, const float timestamp,
      const std::shared_ptr<VisualObject> &obj_ptr) const;

  common::math::ExtendedKalmanFilter<float, 4, 3, 1> InitObjectFilter(
      const float x, const float y, const float theta, const float v) const;

  Eigen::Matrix4f UpdateTransitionMatrix(const double theta, const double v,
      const double dt) const;

  std::unordered_map<int, ObjectFilter> tracked_filters_;

  // @brief Destroy old filters
  void Destroy();

  DISALLOW_COPY_AND_ASSIGN(ObjectCameraExtendedKalmanFilter);
};

}  // namespace perception
}  // namespace apollo

#endif
// MODULES_PERCEPTION_OBSTACLE_CAMERA_FILTER_OBJECT_CAMERA_KALMAN_FILTER_H_
