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
 * @file decision_analyzer.h
 **/

#ifndef MODULES_PLANNING_LATTICE_BEHAVIOR_DECIDER_BEHAVIOR_DECIDER_H
#define MODULES_PLANNING_LATTICE_BEHAVIOR_DECIDER_BEHAVIOR_DECIDER_H

#include "modules/planning/lattice/behavior_decider/behavior_decider.h"

#include "gflags/gflags.h"
#include "modules/planning/lattice/behavior_decider/scenario_manager.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/lattice/util/lattice_params.h"
#include "modules/planning/lattice/behavior_decider/path_time_neighborhood.h"
#include "modules/planning/lattice/behavior_decider/condition_filter.h"
#include "modules/planning/lattice/util/reference_line_matcher.h"
#include "modules/planning/lattice/util/lattice_util.h"
#include "modules/common/log.h"
#include "modules/common/proto/geometry.pb.h"

namespace apollo {
namespace planning {

using apollo::common::PointENU;
using apollo::common::PathPoint;

BehaviorDecider::BehaviorDecider() {}

PlanningTarget BehaviorDecider::Analyze(
    Frame* frame,
    ReferenceLineInfo* const reference_line_info,
    const common::TrajectoryPoint& init_planning_point,
    const std::array<double, 3>& lon_init_state,
    const std::vector<common::PathPoint>& discretized_reference_line) {

  CHECK(frame != nullptr);
  CHECK_GT(discretized_reference_line.size(), 0);

  std::vector<PlanningTarget> scenario_decisions;

  if (0 !=
      ScenarioManager::instance()->ComputeWorldDecision(
          frame, reference_line_info,
          init_planning_point, lon_init_state,
          discretized_reference_line, &scenario_decisions)) {
    AERROR << "ComputeWorldDecision error!";
  }

  PlanningTarget ret;
  ret.CopyFrom(scenario_decisions[0]);
  for (const auto& reference_point : discretized_reference_line) {
    ret.mutable_discretized_reference_line()
        ->add_discretized_reference_line_point()
        ->CopyFrom(reference_point);
  }

  if (StopDecisionNearDestination(frame, lon_init_state,
                                  discretized_reference_line, &ret)) {
    AINFO << "STOP decision when near the routing end.";
    return ret;
  } else {
    AINFO << "GO decision made";
  }

  PathTimeNeighborhood path_time_neighborhood(frame, lon_init_state,
      reference_line_info->reference_line(),
      discretized_reference_line);

  ConditionFilter condition_filter(frame, lon_init_state, speed_limit,
      reference_line_info->reference_line(), discretized_reference_line);

  std::vector<SampleBound> sample_bounds =
      condition_filter.QuerySampleBounds();

  // Debug SampleBound
  AINFO << "[Printing SampleBound]";
  if (sample_bounds.empty()) {
    AINFO << " ------ sample_bounds empty";
  } else {
    for (const SampleBound& sample_bound : sample_bounds) {
      AINFO << " ------ sample_bound: " << sample_bound.ShortDebugString();
    }
  }

  if (sample_bounds.empty()) {
    ret.set_decision_type(PlanningTarget::CRUISE);
    ret.set_cruise_speed(FLAGS_default_cruise_speed);
    return ret;
  }

  for (const auto& sample_bound : sample_bounds) {
    ret.add_sample_bound()->CopyFrom(sample_bound);
  }
  ret.set_decision_type(PlanningTarget::CRUISE);
  ret.set_cruise_speed(FLAGS_default_cruise_speed);
  return ret;
}

bool BehaviorDecider::StopDecisionNearDestination(
    Frame* frame, const std::array<double, 3>& lon_init_state,
    const std::vector<common::PathPoint>& discretized_reference_line,
    PlanningTarget* planning_target) {
  PointENU routing_end = frame->GetRoutingDestination();
  PathPoint routing_end_matched_point =
      ReferenceLineMatcher::MatchToReferenceLine(
          discretized_reference_line, routing_end.x(), routing_end.y());

  double dist_x = routing_end.x() - routing_end_matched_point.x();
  double dist_y = routing_end.y() - routing_end_matched_point.y();
  double dist = std::hypot(dist_x, dist_y);
  if (dist > dist_thred_omit_routing_end) {
    return false;
  }

  double res_s = routing_end_matched_point.s() - lon_init_state[0];
  if (res_s <= stop_margin) {
    planning_target->set_decision_type(PlanningTarget::STOP);
    planning_target->set_stop_point(routing_end_matched_point.s());
    return true;
  } else {
    double v = lon_init_state[1];
    double required_stop_deceleration = (v * v) / (2.0 * res_s);

    if (required_stop_deceleration > stop_acc_thred) {
      planning_target->set_decision_type(PlanningTarget::STOP);
      planning_target->set_stop_point(routing_end_matched_point.s());
      return true;
    } else {
      AINFO << "required_stop_deceleration requirement not satisfied";
    }
  }
  return false;
}

void BehaviorDecider::GetNearbyObstacles(
    const common::TrajectoryPoint& init_planning_point, const Frame* frame,
    const std::vector<common::PathPoint>& discretized_reference_line,
    std::array<double, 3>* forward_state,
    std::array<double, 3>* backward_state) {
  // TODO(kechxu) implement
}

}  // namespace planning
}  // namespace apollo

#endif /* MODULES_PLANNING_DECISION_ANALYZER_H */
