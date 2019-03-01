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
 * @file piecewise_jerk_path_optimizer.h
 **/

#pragma once

#include "modules/planning/tasks/optimizers/path_optimizer.h"

namespace apollo {
namespace planning {

class PiecewiseJerkPathOptimizer : public PathOptimizer {
 public:
  explicit PiecewiseJerkPathOptimizer(const TaskConfig& config);

  virtual ~PiecewiseJerkPathOptimizer() = default;

 private:
  common::Status Process(const SpeedData& speed_data,
                         const ReferenceLine& reference_line,
                         const common::TrajectoryPoint& init_point,
                         PathData* const path_data) override;

  double AdjustLateralDerivativeBounds(const double s_dot,
      const double dl, const double ddl,
      const double l_dot_bounds) const;
};

}  // namespace planning
}  // namespace apollo
