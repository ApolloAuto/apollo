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

#include "modules/planning/tasks/traffic_decider/cipv.h"

#include <string>

#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::util::WithinBound;
using apollo::hdmap::PathOverlap;

CIPV::CIPV(const RuleConfig& config) : TrafficRule(config) {}

bool CIPV::ApplyRule(Frame* frame, ReferenceLineInfo* reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  auto cipv_info = reference_line_info->path_decision()->cipv_info();
  if (!cipv_info.has_cipv_id()) {
    return true;
  }
  for (auto* obstacle :
       reference_line_info->path_decision()->path_obstacles().Items()) {
    if (obstacle->obstacle()->PerceptionId() != cipv_info.cipv_id()) {
      continue;
    }
    auto* path_obstacle =
        reference_line_info->path_decision()->Find(obstacle->Id());
    ObjectDecisionType follow_decision;
    if (CreateFollowDecision(*obstacle, reference_line_info,
                             &follow_decision)) {
      path_obstacle->AddLongitudinalDecision("CIPV", follow_decision);
    }
  }
  return true;
}

bool CIPV::CreateFollowDecision(const PathObstacle& path_obstacle,
                                const ReferenceLineInfo* reference_line_info,
                                ObjectDecisionType* const follow_decision) {
  DCHECK_NOTNULL(follow_decision);

  auto init_point = reference_line_info->AdcPlanningPoint();

  const double follow_speed = init_point.v();
  const double follow_distance_s = -std::fmax(
      follow_speed * FLAGS_follow_time_buffer, FLAGS_follow_min_distance);

  const auto& boundary = path_obstacle.reference_line_st_boundary();
  const auto adc_sl_boundary = reference_line_info->AdcSlBoundary();

  const double reference_s =
      adc_sl_boundary.end_s() + boundary.min_s() + follow_distance_s;
  const double main_stop_s =
      reference_line_info->path_decision().stop_reference_line_s();
  if (main_stop_s < reference_s) {
    ADEBUG << "Follow reference_s is further away, ignore.";
    return false;
  }

  auto ref_point =
      reference_line_info->reference_line().GetReferencePoint(reference_s);

  // set FOLLOW decision
  auto* follow = follow_decision->mutable_follow();
  follow->set_distance_s(follow_distance_s);
  auto* fence_point = follow->mutable_fence_point();
  fence_point->set_x(ref_point.x());
  fence_point->set_y(ref_point.y());
  fence_point->set_z(0.0);
  follow->set_fence_heading(ref_point.heading());

  perception::PerceptionObstacle::Type obstacle_type =
      path_obstacle.obstacle()->Perception().type();
  ADEBUG << "CVPI FOLLOW: obstacle_id[" << path_obstacle.obstacle()->Id()
         << "] obstacle_type[" << PerceptionObstacle_Type_Name(obstacle_type)
         << "]";

  return true;
}

}  // namespace planning
}  // namespace apollo
