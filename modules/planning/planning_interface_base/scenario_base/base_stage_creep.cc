/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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
 * @file base_stage_creep.cc
 **/

#include "modules/planning/planning_interface_base/scenario_base/base_stage_creep.h"

#include <vector>

#include "modules/planning/planning_interface_base/scenario_base/proto/creep_stage.pb.h"

#include "cyber/common/log.h"
#include "modules/common/status/status.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/planning_base/common/reference_line_info.h"
#include "modules/planning/planning_base/common/util/common.h"

namespace apollo {
namespace planning {

using apollo::common::Status;
using apollo::hdmap::PathOverlap;

Status BaseStageCreep::ProcessCreep(
    Frame* frame, ReferenceLineInfo* reference_line_info) const {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  double overlap_end_s = 0.0;
  std::string current_overlap_id;
  bool is_get_overlap_info = GetOverlapStopInfo(
      frame, reference_line_info, &overlap_end_s, &current_overlap_id);

  if (is_get_overlap_info) {
    std::string virtual_obstacle_id = CREEP_VO_ID_PREFIX_ + current_overlap_id;
    const double creep_stop_s =
        GetCreepTargetS(overlap_end_s, *frame, *reference_line_info);
    const std::vector<std::string> wait_for_obstacles;
    util::BuildStopDecision(virtual_obstacle_id, creep_stop_s, 0.0,
                            StopReasonCode::STOP_REASON_CREEPER,
                            wait_for_obstacles, "CreepDecider", frame,
                            reference_line_info);
  }

  return Status::OK();
}

double BaseStageCreep::GetCreepTargetS(
    double overlap_end_s, const Frame& frame,
    const ReferenceLineInfo& reference_line_info) const {
  // todo: more delicate design of creep target distance
  return overlap_end_s + 4.0;
}

double BaseStageCreep::GetCreepFinishS(
    double overlap_end_s, const Frame& frame,
    const ReferenceLineInfo& reference_line_info) const {
  // todo: more delicate design of creep finish distance
  return GetCreepTargetS(overlap_end_s, frame, reference_line_info) - 2.0;
}

bool BaseStageCreep::CheckCreepDone(
    const Frame& frame, const ReferenceLineInfo& reference_line_info,
    const double traffic_sign_overlap_end_s, const double wait_time_sec,
    const double timeout_sec) {
  const auto& creep_config = GetCreepStageConfig();
  bool creep_done = false;
  double creep_stop_s =
      GetCreepFinishS(traffic_sign_overlap_end_s, frame, reference_line_info);

  const double distance =
      creep_stop_s - reference_line_info.AdcSlBoundary().end_s();
  if (distance < 0.0 || wait_time_sec >= timeout_sec) {
    bool all_far_away = true;
    for (auto* obstacle :
         reference_line_info.path_decision().obstacles().Items()) {
      if (obstacle->IsVirtual() || obstacle->IsStatic()) {
        continue;
      }
      if (obstacle->reference_line_st_boundary().min_t() <
          creep_config.min_boundary_t()) {
        const double kepsilon = 1e-6;
        double obstacle_traveled_s =
            obstacle->reference_line_st_boundary().bottom_left_point().s() -
            obstacle->reference_line_st_boundary().bottom_right_point().s();
        ADEBUG << "obstacle[" << obstacle->Id() << "] obstacle_st_min_t["
               << obstacle->reference_line_st_boundary().min_t()
               << "] obstacle_st_min_s["
               << obstacle->reference_line_st_boundary().min_s()
               << "] obstacle_traveled_s[" << obstacle_traveled_s << "]";

        // ignore the obstacle which is already on reference line and moving
        // along the direction of ADC
        if (obstacle_traveled_s < kepsilon &&
            obstacle->reference_line_st_boundary().min_t() <
                creep_config.ignore_max_st_min_t() &&
            obstacle->reference_line_st_boundary().min_s() >
                creep_config.ignore_min_st_min_s()) {
          continue;
        }
        all_far_away = false;
        break;
      }
    }
    auto* creep_decider_status = injector_->planning_context()
                                     ->mutable_planning_status()
                                     ->mutable_creep_decider();
    int creep_clear_counter = creep_decider_status->creep_clear_counter();
    creep_clear_counter = all_far_away ? creep_clear_counter + 1 : 0;
    if (creep_clear_counter >= 5) {
      creep_clear_counter = 0;  // reset
      creep_done = true;
    }
    // use PlanningContext instead of static counter for multi-ADC
    creep_decider_status->set_creep_clear_counter(creep_clear_counter);
  }

  return creep_done;
}

}  // namespace planning
}  // namespace apollo
