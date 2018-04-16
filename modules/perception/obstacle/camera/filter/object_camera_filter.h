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

// Filter module for applicable attributes of each object
// Update state measurement, and derive velocity

#ifndef MODULES_PERCEPTION_OBSTACLE_CAMERA_FILTER_OBJECT_H_
#define MODULES_PERCEPTION_OBSTACLE_CAMERA_FILTER_OBJECT_H_

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "Eigen/Geometry"
#include "opencv2/opencv.hpp"
#include "yaml-cpp/yaml.h"

#include "modules/common/math/kalman_filter_1d.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/obstacle/camera/common/visual_object.h"
#include "modules/perception/obstacle/camera/interface/base_camera_filter.h"

namespace apollo {
namespace perception {

class ObjectCameraFilter : public BaseCameraFilter {
 public:
  ObjectCameraFilter() : BaseCameraFilter() {}

  virtual ~ObjectCameraFilter() {}

  bool Init() override;

  bool Filter(const double &timestamp,
              std::vector<std::shared_ptr<VisualObject>> *objects) override;

  std::string Name() const override;

 private:
  class ObjectFilter {
   public:
    int track_id_ = -1;
    int lost_frame_cnt_ = 0;
    double last_timestamp_ = 0.0f;

    common::math::KalmanFilter1D x_;
    common::math::KalmanFilter1D y_;
    common::math::KalmanFilter1D theta_;
  };

  std::unordered_map<int, ObjectFilter> tracked_filters_;
  const int kMaxKeptFrameCnt = 5;

  // @brief Create filters for new track ids
  void Create(const int &track_id, const double &timestamp,
              const std::shared_ptr<VisualObject> &obj_ptr);

  // @brief Predict step
  void Predict(const int &track_id, const double &timestamp);

  // @brief Update step
  void Update(const int &track_id,
              const std::shared_ptr<VisualObject> &obj_ptr);

  // @brief Get output of estimated state
  void GetState(const int &track_id, std::shared_ptr<VisualObject> obj_ptr);

  // @brief Destroy old filters
  void Destroy();

  DISALLOW_COPY_AND_ASSIGN(ObjectCameraFilter);
};

// Register plugin
REGISTER_CAMERA_FILTER(ObjectCameraFilter);

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_CAMERA_FILTER_OBJECT_H_
