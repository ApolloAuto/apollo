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

#include <limits>
#include <vector>

#include "modules/common/proto/pnc_point.pb.h"

#include "modules/map/hdmap/hdmap_common.h"
#include "modules/planning/tasks/traffic_decider/keep_clear.h"

namespace apollo {
namespace planning {

using apollo::hdmap::PathOverlap;

KeepClear::KeepClear(const TrafficRuleConfig& config) : TrafficRule(config) {}

bool KeepClear::ApplyRule(Frame* const frame,
                          ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  // keep_clear zone
  const std::vector<PathOverlap>& keep_clear_overlaps =
      reference_line_info->reference_line().map_path().clear_area_overlaps();
  for (const auto& keep_clear_overlap : keep_clear_overlaps) {
    const auto obstacle_id =
        KEEP_CLEAR_VO_ID_PREFIX + keep_clear_overlap.object_id;
    if (BuildKeepClearObstacle(frame, reference_line_info,
                               const_cast<PathOverlap*>(&keep_clear_overlap),
                               obstacle_id)) {
      ADEBUG << "KEEP_CLAER for keep_clear_zone["
             << keep_clear_overlap.object_id << "] s["
             << keep_clear_overlap.start_s << ", " << keep_clear_overlap.end_s
             << "] BUILD";
    }
  }

  // junction
  const std::vector<PathOverlap>& junction_overlaps =
      reference_line_info->reference_line().map_path().junction_overlaps();
  for (const auto& junction_overlap : junction_overlaps) {
    const auto obstacle_id =
        KEEP_CLEAR_JUNCTION_VO_ID_PREFIX + junction_overlap.object_id;
    if (BuildKeepClearObstacle(frame, reference_line_info,
                               const_cast<PathOverlap*>(&junction_overlap),
                               obstacle_id)) {
      ADEBUG << "KEEP_CLAER for junction[" << junction_overlap.object_id
             << "] s[" << junction_overlap.start_s << ", "
             << junction_overlap.end_s << "] BUILD";
    }
  }

  return true;
}

bool KeepClear::BuildKeepClearObstacle(
    Frame* const frame, ReferenceLineInfo* const reference_line_info,
    PathOverlap* const keep_clear_overlap,
    const std::string& virtual_obstacle_id) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);
  CHECK_NOTNULL(keep_clear_overlap);

  // check
  const double adc_front_edge_s = reference_line_info->AdcSlBoundary().end_s();
  if (adc_front_edge_s - keep_clear_overlap->start_s >
      config_.keep_clear().min_pass_s_distance()) {
    ADEBUG << "adc inside keep_clear zone[" << keep_clear_overlap->object_id
           << "] s[" << keep_clear_overlap->start_s << ", "
           << keep_clear_overlap->end_s << "] adc_front_edge_s["
           << adc_front_edge_s << "]. skip this keep clear zone";
    return false;
  }

  // create virtual static obstacle
  auto* obstacle = frame->CreateStaticObstacle(
      reference_line_info, virtual_obstacle_id, keep_clear_overlap->start_s,
      keep_clear_overlap->end_s);
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
      StBoundary::BoundaryType::KEEP_CLEAR);

  return true;
}

}  // namespace planning
}  // namespace apollo
