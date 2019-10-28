/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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
 * @file piecewise_jerk_speed_optimizer.h
 **/

#pragma once

#include "modules/planning/tasks/optimizers/speed_optimizer.h"

namespace apollo {
namespace planning {

class PiecewiseJerkSpeedOptimizer : public SpeedOptimizer {
 public:
  explicit PiecewiseJerkSpeedOptimizer(const TaskConfig& config);

  virtual ~PiecewiseJerkSpeedOptimizer() = default;

 private:
  common::Status Process(const PathData& path_data,
                         const common::TrajectoryPoint& init_point,
                         SpeedData* const speed_data) override;
};

}  // namespace planning
}  // namespace apollo
