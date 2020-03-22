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

#include "modules/planning/tasks/deciders/creep_decider/creep_decider.h"

#include <string>
#include <vector>

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/util/common.h"
#include "modules/planning/scenarios/util/util.h"

namespace apollo {
namespace planning {

using apollo::common::Status;
using apollo::hdmap::PathOverlap;

uint32_t CreepDecider::creep_clear_counter_ = 0;

CreepDecider::CreepDecider(const TaskConfig& config) : Decider(config) {
  ACHECK(config_.has_creep_decider_config());
}

Status CreepDecider::Process(Frame* frame,
                             ReferenceLineInfo* reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  double stop_line_s = 0.0;
  std::string current_overlap_id;

  // stop sign
  const std::string stop_sign_overlap_id = PlanningContext::Instance()
                                               ->planning_status()
                                               .stop_sign()
                                               .current_stop_sign_overlap_id();
  // traffic light
  std::string current_traffic_light_overlap_id;
  if (PlanningContext::Instance()
          ->planning_status()
          .traffic_light()
          .current_traffic_light_overlap_id_size() > 0) {
    current_traffic_light_overlap_id = PlanningContext::Instance()
                                           ->planning_status()
                                           .traffic_light()
                                           .current_traffic_light_overlap_id(0);
  }

  // yield sign
  std::string yield_sign_overlap_id;
  if (PlanningContext::Instance()
          ->planning_status()
          .yield_sign()
          .current_yield_sign_overlap_id_size() > 0) {
    yield_sign_overlap_id = PlanningContext::Instance()
                                ->planning_status()
                                .yield_sign()
                                .current_yield_sign_overlap_id(0);
  }

  if (!stop_sign_overlap_id.empty()) {
    // get overlap along reference line
    PathOverlap* current_stop_sign_overlap =
        scenario::util::GetOverlapOnReferenceLine(*reference_line_info,
                                                  stop_sign_overlap_id,
                                                  ReferenceLineInfo::STOP_SIGN);
    if (current_stop_sign_overlap) {
      stop_line_s = current_stop_sign_overlap->end_s +
                    FindCreepDistance(*frame, *reference_line_info);
      current_overlap_id = current_stop_sign_overlap->object_id;
    }
  } else if (!current_traffic_light_overlap_id.empty()) {
    // get overlap along reference line
    PathOverlap* current_traffic_light_overlap =
        scenario::util::GetOverlapOnReferenceLine(
            *reference_line_info, current_traffic_light_overlap_id,
            ReferenceLineInfo::SIGNAL);
    if (current_traffic_light_overlap) {
      stop_line_s = current_traffic_light_overlap->end_s +
                    FindCreepDistance(*frame, *reference_line_info);
      current_overlap_id = current_traffic_light_overlap_id;
    }
  } else if (!yield_sign_overlap_id.empty()) {
    // get overlap along reference line
    PathOverlap* current_yield_sign_overlap =
        scenario::util::GetOverlapOnReferenceLine(
            *reference_line_info, yield_sign_overlap_id,
            ReferenceLineInfo::YIELD_SIGN);
    if (current_yield_sign_overlap) {
      stop_line_s = current_yield_sign_overlap->end_s +
                    FindCreepDistance(*frame, *reference_line_info);
      current_overlap_id = yield_sign_overlap_id;
    }
  }

  if (stop_line_s > 0.0) {
    std::string virtual_obstacle_id = CREEP_VO_ID_PREFIX + current_overlap_id;
    const double creep_stop_s =
        stop_line_s + FindCreepDistance(*frame, *reference_line_info);
    const std::vector<std::string> wait_for_obstacles;
    util::BuildStopDecision(virtual_obstacle_id, creep_stop_s,
                            config_.creep_decider_config().stop_distance(),
                            StopReasonCode::STOP_REASON_CREEPER,
                            wait_for_obstacles, "CreepDecider", frame,
                            reference_line_info);
  }

  return Status::OK();
}

double CreepDecider::FindCreepDistance(
    const Frame& frame, const ReferenceLineInfo& reference_line_info) {
  // more delicate design of creep distance
  return 2.0;
}

bool CreepDecider::CheckCreepDone(const Frame& frame,
                                  const ReferenceLineInfo& reference_line_info,
                                  const double traffic_sign_overlap_end_s,
                                  const double wait_time_sec,
                                  const double timeout_sec) {
  const auto& creep_config = config_.creep_decider_config();
  bool creep_done = false;
  double creep_stop_s = traffic_sign_overlap_end_s +
                        FindCreepDistance(frame, reference_line_info);

  const double distance =
      creep_stop_s - reference_line_info.AdcSlBoundary().end_s();
  if (distance < creep_config.max_valid_stop_distance() ||
      wait_time_sec >= timeout_sec) {
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

    creep_clear_counter_ = all_far_away ? creep_clear_counter_ + 1 : 0;
    if (creep_clear_counter_ >= 5) {
      creep_clear_counter_ = 0;  // reset
      creep_done = true;
    }
  }
  return creep_done;
}

}  // namespace planning
}  // namespace apollo
