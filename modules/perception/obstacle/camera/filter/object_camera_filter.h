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
#include <string>
#include <vector>

#include "Eigen/Geometry"
#include "opencv2/opencv.hpp"
#include "yaml-cpp/yaml.h"

#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/obstacle/camera/common/visual_object.h"
#include "modules/perception/obstacle/camera/interface/base_camera_filter.h"
#include "modules/perception/obstacle/camera/filter/kalman_filter_1d.h"

namespace apollo {
namespace perception {

class ObjectCameraFilter : public BaseCameraFilter {
 public:
  ObjectCameraFilter() : BaseCameraFilter() {}

  virtual ~ObjectCameraFilter() {}

  bool Init() override;

  bool Filter(std::vector<VisualObjectPtr>* objects) override;

  std::string Name() const override;

 private:
  class ObjectFilter {
   public:
    int track_id = -1;
    int lost_frame_count = 0;
    float last_seen_timestamp = 0.0f;

    KalmanFilter1D x_;
    KalmanFilter1D y_;
    KalmanFilter1D z_;

    KalmanFilter1D alpha_;
    KalmanFilter1D theta_;

    KalmanFilter1D length_;
    KalmanFilter1D width_;
    KalmanFilter1D height_;

    // TODO(later) tune and put in config
    ObjectFilter() {
      x_.Init();
      y_.Init();
      z_.Init();
      alpha_.Init();
      theta_.Init();
      length_.Init();
      width_.Init();
      height_.Init();
    }
  };

  std::map<int, ObjectFilter> tracked_filters_;

  DISALLOW_COPY_AND_ASSIGN(ObjectCameraFilter);
};

// Register plugin
REGISTER_CAMERA_FILTER(ObjectCameraFilter);

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_CAMERA_FILTER_OBJECT_H_
