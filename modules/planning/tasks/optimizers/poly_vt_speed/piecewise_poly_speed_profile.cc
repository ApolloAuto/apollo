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
 * @file piecewise_poly_speed_profile.cc
 **/

#include "modules/planning/tasks/optimizers/poly_vt_speed/piecewise_poly_speed_profile.h"

namespace apollo {
namespace planning {

PiecewisePolySpeedProfile::PiecewisePolySpeedProfile(
    const PiecewisePolySpeedCurve& curve)
    : curve_(curve) {}

PiecewisePolySpeedProfile::PiecewisePolySpeedProfile(
    const PiecewisePolySpeedCurve& curve, const size_t num_eval_points)
    : curve_(curve) {
  curve_.SampleSpeedPoints(num_eval_points, &eval_points_);
}

void PiecewisePolySpeedProfile::set_curve(
    const PiecewisePolySpeedCurve& curve) {
  curve_ = curve;
}

void PiecewisePolySpeedProfile::set_cost(const double cost) { cost_ = cost; }

void PiecewisePolySpeedProfile::set_collision(const bool collision) {
  collision_ = collision;
}

void PiecewisePolySpeedProfile::GeneratePoints(
    const size_t num_eval_points) {
  curve_.SampleSpeedPoints(num_eval_points, &eval_points_);
}

void PiecewisePolySpeedProfile::GeneratePointsByTime(
    const double t_res) {
  curve_.SampleSpeedPointsWithTime(t_res, &eval_points_);
}

const std::vector<common::SpeedPoint>& PiecewisePolySpeedProfile::eval_points()
    const {
  return eval_points_;
}

double PiecewisePolySpeedProfile::cost() const { return cost_; }

bool PiecewisePolySpeedProfile::collision() const { return collision_; }
}  // namespace planning
}  // namespace apollo
