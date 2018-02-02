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

#include "modules/planning/tasks/traffic_decider/keep_clear.h"

#include <limits>
#include <vector>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/map/hdmap/hdmap_common.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::hdmap::PathOverlap;

KeepClear::KeepClear(const RuleConfig& config) : TrafficRule(config) {}

bool KeepClear::ApplyRule(Frame* const frame,
                          ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  if (!FLAGS_enable_clear_area) {
    return true;
  }

  const std::vector<PathOverlap>& clear_area_overlaps =
      reference_line_info->reference_line().map_path().clear_area_overlaps();
  for (const auto& clear_area_overlap : clear_area_overlaps) {
    BuildClearAreaObstacle(frame, reference_line_info,
                           const_cast<PathOverlap*>(&clear_area_overlap));
    ADEBUG << "clear_area[" << clear_area_overlap.object_id << "] BUILD";
  }

  return true;
}

bool KeepClear::BuildClearAreaObstacle(
    Frame* const frame,
    ReferenceLineInfo* const reference_line_info,
    PathOverlap* const clear_area_overlap) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);
  CHECK_NOTNULL(clear_area_overlap);

  const double adc_front_edge_s = reference_line_info->AdcSlBoundary().end_s();
  if (clear_area_overlap->start_s < adc_front_edge_s) {
    ADEBUG << "adc is already inside clear area["
           << clear_area_overlap->object_id << "]. skip this clear area";
    return true;
  }

  common::SLPoint sl_point;
  sl_point.set_s(clear_area_overlap->start_s);
  sl_point.set_l(0.0);
  common::math::Vec2d start_xy;
  const auto& reference_line = reference_line_info->reference_line();
  if (!reference_line.SLToXY(sl_point, &start_xy)) {
    AERROR << "Failed to get xy from sl: " << sl_point.DebugString();
    return false;
  }

  double left_lane_width = 0.0;
  double right_lane_width = 0.0;
  if (!reference_line.GetLaneWidth(clear_area_overlap->start_s,
                                   &left_lane_width,
                                   &right_lane_width)) {
    AERROR << "Failed to get lane width at s["
        << clear_area_overlap->start_s << "]";
    return false;
  }
  common::math::Vec2d end_xy;
  sl_point.set_s(clear_area_overlap->end_s);
  if (!reference_line_info->reference_line().SLToXY(sl_point, &end_xy)) {
    AERROR << "Failed to get xy from sl: " << sl_point.DebugString();
    return false;
  }
  common::math::Box2d clear_area_box{
      common::math::LineSegment2d(start_xy, end_xy),
      left_lane_width + right_lane_width};
  const auto obstacle_id =
      FLAGS_clear_area_virtual_object_id_prefix + clear_area_overlap->object_id;
  auto* obstacle =
      frame->AddStaticVirtualObstacle(obstacle_id, clear_area_box);
  if (!obstacle) {
    AERROR << "Failed to create obstacle " << obstacle_id << " in frame";
    return false;
  }
  auto* path_obstacle = reference_line_info->AddObstacle(obstacle);
  if (!path_obstacle) {
    AERROR << "Failed to create path_obstacle for " << obstacle_id;
    return false;
  }
  path_obstacle->SetStBoundaryType(StBoundary::BoundaryType::KEEP_CLEAR);

  return true;
}

}  // namespace planning
}  // namespace apollo
