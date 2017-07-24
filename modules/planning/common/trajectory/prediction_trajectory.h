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
 * @file prediction_trajectory.h
 **/

#ifndef MODULES_PLANNING_COMMON_TRAJECTORY_PREDICTION_TRAJECTORY_H
#define MODULES_PLANNING_COMMON_TRAJECTORY_PREDICTION_TRAJECTORY_H

#include "modules/planning/common/trajectory/discretized_trajectory.h"

namespace apollo {
namespace planning {

using apollo::common::TrajectoryPoint;

class PredictionTrajectory : public DiscretizedTrajectory {
 public:
  double probability() const;

  double start_timestamp() const;

  void set_probability(const double prob);

  void set_start_timestamp(const double ts);

  TrajectoryPoint* trajectory_point_ptr(const std::uint32_t index);

 private:
  double _probability;
  double _start_timestamp;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_COMMON_TRAJECTORY_PREDICTION_TRAJECTORY_H
