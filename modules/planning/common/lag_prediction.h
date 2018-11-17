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
 **/

#ifndef MODULES_PLANNING_COMMON_LAG_PREDICTION_H_
#define MODULES_PLANNING_COMMON_LAG_PREDICTION_H_

#include <unordered_map>
#include <unordered_set>

#include "modules/prediction/proto/prediction_obstacle.pb.h"

namespace apollo {
namespace planning {

class LagPrediction {
 public:
  LagPrediction(uint32_t min_appear_num, uint32_t max_disappear_num);
  void GetLaggedPrediction(prediction::PredictionObstacles* obstacles) const;

  struct LagInfo {
    uint32_t last_observed_seq = 0;
    double last_observed_time = 0.0;
    uint32_t count = 0;
    const prediction::PredictionObstacle* obstacle_ptr = nullptr;
  };

 private:
  void AddObstacleToPrediction(
      double delay_sec, const prediction::PredictionObstacle& obstacle,
      prediction::PredictionObstacles* obstacles) const;

  uint32_t min_appear_num_ = 0;
  uint32_t max_disappear_num_ = 0;
};

}  // namespace planning
}  // namespace apollo

#endif  // namespace MODULES_PLANNING_COMMON_LAG_PREDICTION_H_
