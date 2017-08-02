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
 * @file trjactory_cost.h
 **/

#include "modules/planning/optimizer/dp_poly_path/trajectory_cost.h"

#include <algorithm>
#include <cmath>

#include "modules/common/math/vec2d.h"
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using Box2d = ::apollo::common::math::Box2d;
using Vec2d = ::apollo::common::math::Vec2d;
using TrajectoryPoint = ::apollo::common::TrajectoryPoint;

TrajectoryCost::TrajectoryCost(const DpPolyPathConfig &config,
                               const ReferenceLine &reference_line,
                               const common::VehicleParam &vehicle_param,
                               const SpeedData &heuristic_speed_data,
                               const DecisionData &decision_data)
    : config_(config),
      reference_line_(&reference_line),
      vehicle_param_(vehicle_param),
      heuristic_speed_data_(heuristic_speed_data) {
  const double total_time =
      std::min(heuristic_speed_data_.total_time(), FLAGS_prediction_total_time);
  evaluate_times_ = static_cast<uint32_t>(
      std::floor(total_time / config.eval_time_interval()));

  // Mapping Static obstacle
  for (const auto ptr_static_obstacle : decision_data.StaticObstacles()) {
    Vec2d center_point = ptr_static_obstacle->Center();
    Box2d obstacle_box = {center_point, ptr_static_obstacle->Heading(),
                          ptr_static_obstacle->BoundingBox().length(),
                          ptr_static_obstacle->BoundingBox().width()};
    static_obstacle_boxes_.push_back(std::move(obstacle_box));
  }

  // Mapping dynamic obstacle
  for (const auto ptr_dynamic_obstacle : decision_data.DynamicObstacles()) {
    for (const auto trajectory :
         ptr_dynamic_obstacle->prediction_trajectories()) {
      std::vector<Box2d> obstacle_by_time;
      for (std::size_t time = 0; time < evaluate_times_ + 1; ++time) {
        TrajectoryPoint traj_point = ptr_dynamic_obstacle->get_point_at(
            trajectory, time * config.eval_time_interval());
        Vec2d center_point = {traj_point.path_point().x(),
                              traj_point.path_point().y()};
        Box2d obstacle_box = {center_point, traj_point.path_point().theta(),
                              ptr_dynamic_obstacle->BoundingBox().length(),
                              ptr_dynamic_obstacle->BoundingBox().width()};
        obstacle_by_time.push_back(std::move(obstacle_box));
      }
      dynamic_obstacle_trajectory_.push_back(std::move(obstacle_by_time));
      dynamic_obstacle_probability_.push_back(trajectory.probability());
    }
  }
  CHECK(dynamic_obstacle_trajectory_.size() ==
        dynamic_obstacle_probability_.size());
}

double TrajectoryCost::calculate(const QuinticPolynomialCurve1d &curve,
                                 const double start_s,
                                 const double end_s) const {
  double total_cost = 0.0;
  // path_cost
  double path_s = 0.0;
  while (path_s < (end_s - start_s)) {
    double l = std::fabs(curve.evaluate(0, path_s));
    total_cost += l;

    double dl = std::fabs(curve.evaluate(1, path_s));
    total_cost += dl;

    path_s += config_.path_resolution();
  }
  // obstacle cost
  return total_cost;
}

}  // namespace planning
}  // namespace apollo
