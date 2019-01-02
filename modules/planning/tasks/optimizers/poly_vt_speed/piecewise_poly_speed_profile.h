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
 * @file piecewise_poly_speed_profile.h
 **/
#pragma once

#include <vector>

#include "modules/planning/tasks/optimizers/poly_vt_speed/piecewise_poly_speed_curve.h"

namespace apollo {
namespace planning {

class PiecewisePolySpeedProfile {
 public:
  PiecewisePolySpeedProfile() = default;
  explicit PiecewisePolySpeedProfile(const PiecewisePolySpeedCurve& curve);
  PiecewisePolySpeedProfile(const PiecewisePolySpeedCurve& curve,
                            const size_t num_eval_points);

  void set_curve(const PiecewisePolySpeedCurve& curve);

  void set_cost(const double cost);

  void set_collision(const bool collision);
  /**
   * @brief: Generate discretized speed points
   */
  void GeneratePoints(const size_t num_eval_points);

  void GeneratePointsByTime(const double t_resolution);

  const std::vector<common::SpeedPoint>& eval_points() const;

  double cost() const;

  bool collision() const;

 private:
  PiecewisePolySpeedCurve curve_;
  std::vector<common::SpeedPoint> eval_points_;
  double cost_;
  bool collision_ = false;
};

}  // namespace planning
}  // namespace apollo
