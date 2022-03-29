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

#include <memory>

#include "modules/planning/common/trajectory1d/piecewise_acceleration_trajectory1d.h"

namespace apollo {
namespace planning {

class PiecewiseBrakingTrajectoryGenerator {
 public:
  PiecewiseBrakingTrajectoryGenerator() = delete;

  static std::shared_ptr<Curve1d> Generate(
      const double s_target, const double s_curr, const double v_target,
      const double v_curr, const double a_comfort, const double d_comfort,
      const double max_time);

  static double ComputeStopDistance(const double v, const double dec);

  static double ComputeStopDeceleration(const double dist, const double v);
};

}  // namespace planning
}  // namespace apollo
