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

#include "modules/planning/traffic_rules/keep_clear.h"

#include <memory>
#include <vector>

#include "modules/common_msgs/basic_msgs/pnc_point.pb.h"
#include "modules/map/hdmap/hdmap_common.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/proto/planning_status.pb.h"

namespace apollo {
namespace planning {

using apollo::common::Status;
using apollo::hdmap::PathOverlap;

KeepClear::KeepClear(const TrafficRuleConfig& config,
                     const std::shared_ptr<DependencyInjector>& injector)
    : TrafficRule(config, injector) {}

Status KeepClear::ApplyRule(Frame* const frame,
                            ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  // keep_clear zone
  if (config_.keep_clear().enable_keep_clear_zone()) {
    const std::vector<PathOverlap>& keep_clear_overlaps =
        reference_line_info->reference_line().map_path().clear_area_overlaps();
    for (const auto& keep_clear_overlap : keep_clear_overlaps) {
      const auto obstacle_id =
          KEEP_CLEAR_VO_ID_PREFIX + keep_clear_overlap.object_id;

      if (BuildKeepClearObstacle(frame, reference_line_info, obstacle_id,
                                 keep_clear_overlap.start_s,
                                 keep_clear_overlap.end_s)) {
        ADEBUG << "KEEP_CLAER for keep_clear_zone["
               << keep_clear_overlap.object_id << "] s["
               << keep_clear_overlap.start_s << ", " << keep_clear_overlap.end_s
               << "] BUILD";
      }
    }
  }

  // junction
  if (config_.keep_clear().enable_junction()) {
    hdmap::PathOverlap* crosswalk_overlap = nullptr;
    hdmap::PathOverlap* stop_sign_overlap = nullptr;
    hdmap::PathOverlap* traffic_light_overlap = nullptr;
    hdmap::PathOverlap* pnc_junction_overlap = nullptr;

    const auto& first_encountered_overlaps =
        reference_line_info->FirstEncounteredOverlaps();
    for (const auto& overlap : first_encountered_overlaps) {
      ADEBUG << overlap.first << ", " << overlap.second.DebugString();
      switch (overlap.first) {
        case ReferenceLineInfo::CROSSWALK:
          ADEBUG << "CROSSWALK[" << overlap.second.object_id << "] s["
                 << overlap.second.start_s << ", " << overlap.second.end_s
                 << "]";
          crosswalk_overlap = const_cast<PathOverlap*>(&overlap.second);
          break;
        case ReferenceLineInfo::STOP_SIGN:
          ADEBUG << "STOP_SIGN[" << overlap.second.object_id << "] s["
                 << overlap.second.start_s << ", " << overlap.second.end_s
                 << "]";
          stop_sign_overlap = const_cast<PathOverlap*>(&overlap.second);
          break;
        case ReferenceLineInfo::SIGNAL:
          ADEBUG << "SIGNAL[" << overlap.second.object_id << "] s["
                 << overlap.second.start_s << ", " << overlap.second.end_s
                 << "]";
          traffic_light_overlap = const_cast<PathOverlap*>(&overlap.second);
          break;
        case ReferenceLineInfo::PNC_JUNCTION:
          ADEBUG << "PNC_JUNCTION[" << overlap.second.object_id << "] s["
                 << overlap.second.start_s << ", " << overlap.second.end_s
                 << "]";
          pnc_junction_overlap = const_cast<PathOverlap*>(&overlap.second);
          break;
        default:
          break;
      }
    }

    if (pnc_junction_overlap != nullptr) {
      const double adc_front_edge_s =
          reference_line_info->AdcSlBoundary().end_s();
      if (!IsCreeping(pnc_junction_overlap->start_s, adc_front_edge_s)) {
        // adjust pnc_junction start_s to align with
        // the start_s of other "stop type" overlaps
        double pnc_junction_start_s = pnc_junction_overlap->start_s;
        // traffic_light, stop_sign, and then crosswalk if neither
        if (traffic_light_overlap != nullptr &&
            std::fabs(pnc_junction_start_s - traffic_light_overlap->start_s) <=
                config_.keep_clear().align_with_traffic_sign_tolerance()) {
          ADEBUG << "adjust pnc_junction_start_s[" << pnc_junction_start_s
                 << "] to traffic_light_start_s"
                 << traffic_light_overlap->start_s << "]";
          pnc_junction_start_s = traffic_light_overlap->start_s;
        } else if (stop_sign_overlap != nullptr &&
                   std::fabs(pnc_junction_start_s -
                             stop_sign_overlap->start_s) <=
                       config_.keep_clear()
                           .align_with_traffic_sign_tolerance()) {
          ADEBUG << "adjust pnc_junction_start_s[" << pnc_junction_start_s
                 << "] to stop_sign_start_s" << stop_sign_overlap->start_s
                 << "]";
          pnc_junction_start_s = stop_sign_overlap->start_s;
        } else if (crosswalk_overlap != nullptr &&
                   std::fabs(pnc_junction_start_s -
                             crosswalk_overlap->start_s) <=
                       config_.keep_clear()
                           .align_with_traffic_sign_tolerance()) {
          ADEBUG << "adjust pnc_junction_start_s[" << pnc_junction_start_s
                 << "] to cross_walk_start_s" << crosswalk_overlap->start_s
                 << "]";
          pnc_junction_start_s = crosswalk_overlap->start_s;
        }

        const auto obstacle_id =
            KEEP_CLEAR_JUNCTION_VO_ID_PREFIX + pnc_junction_overlap->object_id;

        if (BuildKeepClearObstacle(frame, reference_line_info, obstacle_id,
                                   pnc_junction_start_s,
                                   pnc_junction_overlap->end_s)) {
          ADEBUG << "KEEP_CLAER for junction["
                 << pnc_junction_overlap->object_id << "] s["
                 << pnc_junction_start_s << ", " << pnc_junction_overlap->end_s
                 << "] BUILD";
        }
      }
    }
  }

  return Status::OK();
}

bool KeepClear::IsCreeping(const double pnc_junction_start_s,
                           const double adc_front_edge_s) const {
  // check if in scenario creep stage
  // while creeping, no need create keep clear obstacle
  const auto& stage_type =
      injector_->planning_context()->planning_status().scenario().stage_type();
  if (stage_type != StageType::STOP_SIGN_UNPROTECTED_CREEP &&
      stage_type !=
          StageType::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN_CREEP &&
      stage_type != StageType::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN_CREEP) {
    return false;
  }

  // check distance
  static constexpr double kDistance = 5.0;
  return (fabs(adc_front_edge_s - pnc_junction_start_s) <= kDistance);
}

bool KeepClear::BuildKeepClearObstacle(
    Frame* const frame, ReferenceLineInfo* const reference_line_info,
    const std::string& virtual_obstacle_id, const double keep_clear_start_s,
    const double keep_clear_end_s) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  // check
  const double adc_front_edge_s = reference_line_info->AdcSlBoundary().end_s();
  if (adc_front_edge_s - keep_clear_start_s >
      config_.keep_clear().min_pass_s_distance()) {
    ADEBUG << "adc inside keep_clear zone[" << virtual_obstacle_id << "] s["
           << keep_clear_start_s << ", " << keep_clear_end_s
           << "] adc_front_edge_s[" << adc_front_edge_s
           << "]. skip this keep clear zone";
    return false;
  }

  ADEBUG << "keep clear obstacle: [" << keep_clear_start_s << ", "
         << keep_clear_end_s << "]";
  // create virtual static obstacle
  auto* obstacle =
      frame->CreateStaticObstacle(reference_line_info, virtual_obstacle_id,
                                  keep_clear_start_s, keep_clear_end_s);
  if (!obstacle) {
    AERROR << "Failed to create obstacle [" << virtual_obstacle_id << "]";
    return false;
  }
  auto* path_obstacle = reference_line_info->AddObstacle(obstacle);
  if (!path_obstacle) {
    AERROR << "Failed to create path_obstacle: " << virtual_obstacle_id;
    return false;
  }
  path_obstacle->SetReferenceLineStBoundaryType(
      STBoundary::BoundaryType::KEEP_CLEAR);

  return true;
}

}  // namespace planning
}  // namespace apollo
