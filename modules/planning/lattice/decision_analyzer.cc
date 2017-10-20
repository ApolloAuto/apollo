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
 * @file decision_analyzer.cpp
 **/

#include "modules/planning/lattice/decision_analyzer.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "modules/planning/lattice/planning_target.h"
#include "modules/planning/common/planning_gflags.h"

#include "gflags/gflags.h"

namespace apollo {
namespace planning {

DEFINE_double(default_cruise_speed, 5.0, "default cruise speed");

DecisionAnalyzer::DecisionAnalyzer() {}

PlanningTarget DecisionAnalyzer::analyze(
    Frame* frame, const common::TrajectoryPoint& init_planning_point,
    const std::array<double, 3>& lon_init_state,
    const common::PathPoint& matched_reference_line_point,
    const std::vector<common::PathPoint>& reference_line) {
  PlanningTarget planning_objective;
  // TODO(yajia): currently only support cruising with a fake target speed.
  planning_objective.set_task(PlanningTarget::Task::CRUISE);

  // double target_cruise_speed = FLAGS_default_cruise_speed;
  double target_cruise_speed =
      AdjustCruiseSpeed(lon_init_state[0], lon_init_state[1],
                        FLAGS_default_cruise_speed, reference_line);
  planning_objective.set_cruise_target(target_cruise_speed);
  return planning_objective;
}

double DecisionAnalyzer::AdjustCruiseSpeed(
    double init_s, double init_speed, double init_target_speed,
    const std::vector<common::PathPoint>& reference_line) const {
  auto comp = [](const common::PathPoint& point,
                 const double s) { return point.s() < s; };

  auto it_lower = std::lower_bound(reference_line.begin(), reference_line.end(),
                                   init_s, comp);

  double target_speed = init_target_speed;
  double s_acc = it_lower->s() - init_s;
  double forward_distance =
      (init_speed + target_speed) * FLAGS_trajectory_time_length * 0.5;

  double lateral_acceleration_comfort_factor = 1.0;
  double lateral_acceleration_bound =
      lateral_acceleration_comfort_factor * FLAGS_lateral_acceleration_bound;

  while (s_acc < forward_distance && it_lower != reference_line.end()) {
    double kappa_abs = std::abs(it_lower->kappa());
    if (target_speed * target_speed * kappa_abs > lateral_acceleration_bound) {
      target_speed = std::sqrt(lateral_acceleration_bound / kappa_abs);
    }
    s_acc = it_lower->s() - init_s;
    it_lower++;
  }
  return target_speed;
}

}  // namespace planning
}  // namespace apollo
