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

/**
 * @file
 **/

#pragma once

#include <utility>
#include <vector>

#include "modules/common_msgs/basic_msgs/pnc_point.pb.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/trajectory1d/piecewise_jerk_trajectory1d.h"

namespace apollo {
namespace planning {

class LateralQPOptimizer {
 public:
  LateralQPOptimizer() = default;

  virtual ~LateralQPOptimizer() = default;

  virtual bool optimize(
      const std::array<double, 3>& d_state, const double delta_s,
      const std::vector<std::pair<double, double>>& d_bounds) = 0;

  virtual PiecewiseJerkTrajectory1d GetOptimalTrajectory() const;
  virtual std::vector<common::FrenetFramePoint> GetFrenetFramePath() const;

 protected:
  double delta_s_ = FLAGS_default_delta_s_lateral_optimization;

  std::vector<double> opt_d_;

  std::vector<double> opt_d_prime_;

  std::vector<double> opt_d_pprime_;
};

}  // namespace planning
}  // namespace apollo
