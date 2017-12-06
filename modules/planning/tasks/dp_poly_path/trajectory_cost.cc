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

#include "modules/planning/tasks/dp_poly_path/trajectory_cost.h"

#include <algorithm>
#include <cmath>

#include "modules/common/proto/pnc_point.pb.h"

#include "modules/common/math/vec2d.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::math::Box2d;
using apollo::common::math::Vec2d;
using apollo::common::TrajectoryPoint;

TrajectoryCost::TrajectoryCost(
    const DpPolyPathConfig &config, const ReferenceLine &reference_line,
    const std::vector<const PathObstacle *> &obstacles,
    const common::VehicleParam &vehicle_param,
    const SpeedData &heuristic_speed_data, const common::SLPoint &init_sl_point)
    : config_(config),
      reference_line_(&reference_line),
      vehicle_param_(vehicle_param),
      heuristic_speed_data_(heuristic_speed_data),
      init_sl_point_(init_sl_point) {
  const double total_time =
      std::min(heuristic_speed_data_.TotalTime(), FLAGS_prediction_total_time);

  num_of_time_stamps_ = static_cast<uint32_t>(
      std::floor(total_time / config.eval_time_interval()));

  for (const auto ptr_path_obstacle : obstacles) {
    if (ptr_path_obstacle->IsIgnore()) {
      continue;
    }
    const auto ptr_obstacle = ptr_path_obstacle->obstacle();
    if (Obstacle::IsVirtualObstacle(ptr_obstacle->Perception())) {
      // Virtual obstacle
      continue;
    }
    std::vector<Box2d> box_by_time;
    for (uint32_t t = 0; t <= num_of_time_stamps_; ++t) {
      TrajectoryPoint trajectory_point =
          ptr_obstacle->GetPointAtTime(t * config.eval_time_interval());
      Box2d obstacle_box = ptr_obstacle->GetBoundingBox(trajectory_point);
      box_by_time.push_back(obstacle_box);
    }
    obstacle_boxes_.push_back(box_by_time);
  }
}

double TrajectoryCost::CalculatePathCost(const QuinticPolynomialCurve1d &curve,
                                         const double start_s,
                                         const double end_s) const {
  double path_cost = 0.0;
  for (double path_s = 0.0; path_s < (end_s - start_s);
       path_s += config_.path_resolution()) {
    const double l = std::fabs(curve.Evaluate(0, path_s));
    path_cost += l * l * config_.path_l_cost();

    const double dl = std::fabs(curve.Evaluate(1, path_s));
    path_cost += dl * dl * config_.path_dl_cost();

    const double ddl = std::fabs(curve.Evaluate(2, path_s));
    path_cost += ddl * ddl * config_.path_ddl_cost();
  }
  return path_cost;
}

double TrajectoryCost::CalculateObstacleCost(
    const QuinticPolynomialCurve1d &curve, const double start_s,
    const double end_s) const {
  double obstacle_cost = 0.0;

  for (double curr_s = start_s; curr_s <= end_s;
       curr_s += config_.path_resolution()) {
    const double s = curr_s - start_s;  // spline curve s
    const double l = curve.Evaluate(0, s);
    const double dl = curve.Evaluate(1, s);

    common::SLPoint sl;  // ego vehicle sl point
    sl.set_s(curr_s);
    sl.set_l(l);
    Vec2d ego_xy_point;  // ego vehicle xy point
    reference_line_->SLToXY(sl, &ego_xy_point);

    ReferencePoint reference_point = reference_line_->GetReferencePoint(curr_s);
    const double one_minus_kappa_r_d = 1 - reference_point.kappa() * l;
    const double delta_theta = std::atan2(dl, one_minus_kappa_r_d);
    const double theta =
        common::math::NormalizeAngle(delta_theta + reference_point.heading());
    const Box2d ego_box = {ego_xy_point, theta, vehicle_param_.length(),
                           vehicle_param_.width()};
    for (const auto &obstacle_trajectory : obstacle_boxes_) {
      auto &obstacle_box = obstacle_trajectory.back();
      // Simple version: calculate obstacle cost by distance
      const double distance = obstacle_box.DistanceTo(ego_box);
      if (distance > config_.obstacle_ignore_distance()) {
        continue;
      } else if (distance <= config_.obstacle_collision_distance()) {
        obstacle_cost += config_.obstacle_collision_cost();
      } else if (distance <= config_.obstacle_risk_distance()) {
        obstacle_cost += RiskDistanceCost(distance);
      } else {
        obstacle_cost += RegularDistanceCost(distance);
      }
    }
  }
  return obstacle_cost;
}

double TrajectoryCost::Calculate(const QuinticPolynomialCurve1d &curve,
                                 const double start_s,
                                 const double end_s) const {
  double total_cost = 0.0;
  // path cost
  total_cost += CalculatePathCost(curve, start_s, end_s);

  // Obstacle cost
  total_cost += CalculateObstacleCost(curve, start_s, end_s);
  return total_cost;
}

double TrajectoryCost::RiskDistanceCost(const double distance) const {
  return (5.0 - distance) * ((5.0 - distance)) * 10;
}

double TrajectoryCost::RegularDistanceCost(const double distance) const {
  return std::max(20.0 - distance, 0.0);
}

}  // namespace planning
}  // namespace apollo
