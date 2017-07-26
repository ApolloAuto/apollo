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
#include "modules/common/proto/path_point.pb.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

TrajectoryCost::TrajectoryCost(const DpPolyPathConfig &config,
                               const common::VehicleParam &vehicle_param,
                               const SpeedData &heuristic_speed_data,
                               const DecisionData &decision_data)
    : _config(config),
      _vehicle_param(vehicle_param),
      _heuristic_speed_data(heuristic_speed_data) {
  const auto &static_obstacles = decision_data.StaticObstacles();
  const double total_time =
      std::min(_heuristic_speed_data.total_time(), FLAGS_prediction_total_time);
  _evaluate_times = static_cast<uint32_t>(
      std::floor(total_time / config.eval_time_interval()));

  // Mapping Static obstacle
  for (uint32_t i = 0; i < static_obstacles.size(); ++i) {
    std::vector<::apollo::common::math::Box2d> obstacle_by_time;
    ::apollo::common::TrajectoryPoint traj_point =
        static_obstacles[i]->prediction_trajectories()[0].evaluate(0.0);
    ::apollo::common::math::Vec2d center_point = {traj_point.path_point().x(),
                                                  traj_point.path_point().y()};
    ::apollo::common::math::Box2d obstacle_box = {
        center_point, traj_point.path_point().theta(),
        static_obstacles[i]->BoundingBox().length(),
        static_obstacles[i]->BoundingBox().width()};
    for (uint32_t j = 0; i <= _evaluate_times; ++j) {
      obstacle_by_time.push_back(obstacle_box);
    }
    _obstacle_trajectory.push_back(obstacle_by_time);
    _obstacle_probability.push_back(1.0);
  }

  // Mapping dynamic obstacle
  const auto &dynamic_obstacles = decision_data.DynamicObstacles();
  for (uint32_t i = 0; i < dynamic_obstacles.size(); ++i) {
    const auto &trajectories = dynamic_obstacles[i]->prediction_trajectories();
    for (uint32_t j = 0; j < trajectories.size(); ++j) {
      const auto &trajectory = trajectories[j];
      std::vector<::apollo::common::math::Box2d> obstacle_by_time;
      for (uint32_t time = 0; time <= _evaluate_times; ++time) {
        TrajectoryPoint traj_point =
            trajectory.evaluate(time * config.eval_time_interval());
        ::apollo::common::math::Vec2d center_point = {
            traj_point.path_point().x(), traj_point.path_point().y()};
        ::apollo::common::math::Box2d obstacle_box = {
            center_point, traj_point.path_point().theta(),
            static_obstacles[i]->BoundingBox().length(),
            static_obstacles[i]->BoundingBox().width()};
        obstacle_by_time.push_back(obstacle_box);
      }
      _obstacle_trajectory.push_back(obstacle_by_time);
      _obstacle_probability.push_back(trajectory.probability());
    }
  }
  // CHECK(_obstacle_probability.size() == _obstacle_trajectory.size());
}

double TrajectoryCost::calculate(const QuinticPolynomialCurve1d &curve,
                                 const double start_s, const double end_s,
                                 const ReferenceLine &reference_line) const {
  const double length = _vehicle_param.length();
  const double width = _vehicle_param.width();
  double total_cost = 0.0;
  for (uint32_t i = 0; i < _evaluate_times; ++i) {
    double eval_time = i * _config.eval_time_interval();
    SpeedPoint speed_point;
    if (!_heuristic_speed_data.get_speed_point_with_time(eval_time,
                                                         &speed_point) ||
        start_s <= speed_point.s()) {
      continue;
    }
    if (speed_point.s() >= end_s) {
      break;
    }
    double l = curve.evaluate(1, speed_point.s() - start_s);
    total_cost += l;  // need refine l cost;

    ::apollo::common::math::Vec2d car_point = {speed_point.s(), l};
    ReferencePoint reference_point =
        reference_line.get_reference_point(car_point.x());
    ::apollo::common::math::Box2d car_box = {
        car_point, reference_point.heading(), length, width};
    for (uint32_t j = 0; j < _obstacle_trajectory.size(); ++j) {
      ::apollo::common::math::Box2d obstacle_box = _obstacle_trajectory[j][i];
      double distance = car_box.DistanceTo(obstacle_box);
      total_cost += distance * _obstacle_probability[j];  // obstacle cost
    }
  }
  return total_cost;
}

}  // namespace planning
}  // namespace apollo
