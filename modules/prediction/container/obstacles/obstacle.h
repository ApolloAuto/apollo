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

/**
 * @file
 * @brief Obstacle
 */

#ifndef MODULES_PREDICTION_CONTAINER_OBSTACLES_OBSTACLE_H_
#define MODULES_PREDICTION_CONTAINER_OBSTACLES_OBSTACLE_H_

#include <deque>
#include <unordered_map>
#include <vector>
#include <string>
#include <mutex>

#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/prediction/proto/feature.pb.h"

#include "modules/common/math/kalman_filter.h"

namespace apollo {
namespace prediction {

class Obstacle {
 public:
  explicit Obstacle();

  virtual ~Obstacle();

  void Insert(
    const apollo::perception::PerceptionObstacle& perception_obstacle,
    const double timestamp);

 private:
  int id_;
  apollo::perception::PerceptionObstacle::Type type_;
  std::deque<Feature> feature_history_;
  apollo::common::math::KalmanFilter<double, 6, 2, 0> kf_motion_tracker_;
  bool is_motion_tracker_enabled;
  std::unordered_map<std::string,
      apollo::common::math::KalmanFilter<double, 4, 2, 0>> kf_lane_tracker_;
  static std::mutex _mutex;
};

}  // namespace prediction
}  // namespace apollo

 #endif  // MODULES_PREDICTION_CONTAINER_OBSTACLES_OBSTACLE_H_
