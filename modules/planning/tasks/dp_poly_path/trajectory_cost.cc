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
#include <utility>

#include "modules/common/proto/pnc_point.pb.h"

#include "modules/common/math/vec2d.h"
#include "modules/common/util/util.h"
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
    } else if (Obstacle::IsStaticObstacle(ptr_obstacle->Perception())) {
      TrajectoryPoint trajectory_point = ptr_obstacle->GetPointAtTime(0.0);
      Box2d obstacle_box = ptr_obstacle->GetBoundingBox(trajectory_point);
      static_obstacle_boxes_.push_back(std::move(obstacle_box));
    } else {
      std::vector<Box2d> box_by_time;
      for (uint32_t t = 0; t <= num_of_time_stamps_; ++t) {
        TrajectoryPoint trajectory_point =
            ptr_obstacle->GetPointAtTime(t * config.eval_time_interval());
        Box2d obstacle_box = ptr_obstacle->GetBoundingBox(trajectory_point);
        box_by_time.push_back(obstacle_box);
      }
      dynamic_obstacle_boxes_.push_back(std::move(box_by_time));
    }
  }
}

double TrajectoryCost::CalculatePathCost(const QuinticPolynomialCurve1d &curve,
                                         const double start_s,
                                         const double end_s) const {
  double path_cost = 0.0;
  for (double path_s = 0.0; path_s < (end_s - start_s);
       path_s += config_.path_resolution()) {
    const double l = std::fabs(curve.Evaluate(0, path_s));

    std::function<double(const double)> quasi_softmax = [this](const double x) {
      const double l0 = this->config_.path_l_cost_param_l0();
      const double b = this->config_.path_l_cost_param_b();
      const double k = this->config_.path_l_cost_param_k();
      return (b + std::exp(-k * (x - l0))) / (1.0 + std::exp(-k * (x - l0)));
    };

    path_cost += l * l * config_.path_l_cost() * quasi_softmax(l);

    const double dl = std::fabs(curve.Evaluate(1, path_s));
    path_cost += dl * dl * config_.path_dl_cost();

    const double ddl = std::fabs(curve.Evaluate(2, path_s));
    path_cost += ddl * ddl * config_.path_ddl_cost();
  }
  return path_cost * config_.path_resolution();
}

double TrajectoryCost::CalculateStaticObstacleCost(
    const QuinticPolynomialCurve1d &curve, const double start_s,
    const double end_s) const {
  double obstacle_cost = 0.0;

  for (double curr_s = start_s; curr_s <= end_s;
       curr_s += config_.path_resolution()) {
    const double s = curr_s - start_s;  // spline curve s
    const double l = curve.Evaluate(0, s);
    const double dl = curve.Evaluate(1, s);

    const common::SLPoint sl =
        common::util::MakeSLPoint(curr_s, l);  // ego vehicle sl point
    const Box2d ego_box = GetBoxFromSLPoint(sl, dl);
    for (const auto &obstacle_box : static_obstacle_boxes_) {
      obstacle_cost += GetCostBetweenObsBoxes(ego_box, obstacle_box);
    }
  }
  return obstacle_cost * config_.path_resolution();
}

double TrajectoryCost::CalculateDynamicObstacleCost(
    const QuinticPolynomialCurve1d &curve, const double start_s,
    const double end_s) const {
  double obstacle_cost = 0.0;
  double time_stamp = 0.0;

  for (size_t index = 0; index < num_of_time_stamps_;
       ++index, time_stamp += config_.eval_time_interval()) {
    common::SpeedPoint speed_point;
    heuristic_speed_data_.EvaluateByTime(time_stamp, &speed_point);
    if (speed_point.s() < start_s - init_sl_point_.s()) {
      continue;
    }
    if (speed_point.s() > end_s - init_sl_point_.s()) {
      break;
    }

    const double s =
        init_sl_point_.s() + speed_point.s() - start_s;  // s on spline curve
    const double l = curve.Evaluate(0, s);
    const double dl = curve.Evaluate(1, s);

    const common::SLPoint sl =
        common::util::MakeSLPoint(init_sl_point_.s() + speed_point.s(), l);
    const Box2d ego_box = GetBoxFromSLPoint(sl, dl);
    for (const auto &obstacle_trajectory : dynamic_obstacle_boxes_) {
      obstacle_cost +=
          GetCostBetweenObsBoxes(ego_box, obstacle_trajectory.at(index));
    }
  }
  return obstacle_cost * config_.eval_time_interval();
}

double TrajectoryCost::GetCostBetweenObsBoxes(const Box2d &ego_box,
                                              const Box2d &obstacle_box) const {
  // Simple version: calculate obstacle cost by distance
  const double distance = obstacle_box.DistanceTo(ego_box);
  if (distance > config_.obstacle_ignore_distance()) {
    return 0.0;
  }

  double obstacle_cost = 0.0;
  if (distance <= config_.obstacle_collision_distance()) {
    obstacle_cost += config_.obstacle_collision_cost();
  } else if (distance <= config_.obstacle_risk_distance()) {
    obstacle_cost += RiskDistanceCost(distance);
  } else {
    obstacle_cost += RegularDistanceCost(distance);
  }
  return obstacle_cost;
}

Box2d TrajectoryCost::GetBoxFromSLPoint(const common::SLPoint &sl,
                                        const double dl) const {
  Vec2d xy_point;
  reference_line_->SLToXY(sl, &xy_point);

  ReferencePoint reference_point = reference_line_->GetReferencePoint(sl.s());

  const double one_minus_kappa_r_d = 1 - reference_point.kappa() * sl.l();
  const double delta_theta = std::atan2(dl, one_minus_kappa_r_d);
  const double theta =
      common::math::NormalizeAngle(delta_theta + reference_point.heading());
  return Box2d(xy_point, theta, vehicle_param_.length(),
               vehicle_param_.width());
}

double TrajectoryCost::Calculate(const QuinticPolynomialCurve1d &curve,
                                 const double start_s,
                                 const double end_s) const {
  double total_cost = 0.0;
  // path cost
  total_cost += CalculatePathCost(curve, start_s, end_s);

  // Obstacle cost
  total_cost += CalculateStaticObstacleCost(curve, start_s, end_s);
  total_cost += CalculateDynamicObstacleCost(curve, start_s, end_s);
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
