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

#include "modules/planning/tasks/traffic_decider/clear_zone.h"

#include <limits>
#include <vector>

#include "modules/common/proto/pnc_point.pb.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/vehicle_state/vehicle_state.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::adapter::AdapterManager;
using apollo::perception::TrafficLight;
using apollo::perception::TrafficLightDetection;

ClearZone::ClearZone(const RuleConfig& config) : TrafficRule(config) {}

bool ClearZone::ApplyRule(Frame* frame,
                          ReferenceLineInfo* const reference_line_info) {
  frame_ = frame;
  reference_line_info_ = reference_line_info;
  const auto& map_path = reference_line_info_->reference_line().map_path();
  for (const auto& clear_zone : map_path.clear_area_overlaps()) {
    if (!BuildClearZoneObstacle(clear_zone)) {
      AERROR << "Failed to build clear zone : " << clear_zone.object_id;
      return false;
    }
  }
  return true;
}

bool ClearZone::BuildClearZoneObstacle(
    const hdmap::PathOverlap& clear_zone_overlap) {
  const double front_edge_s = reference_line_info_->AdcSlBoundary().end_s();
  if (clear_zone_overlap.start_s < front_edge_s) {
    ADEBUG << "adc is already inside clear zone "
           << clear_zone_overlap.object_id << ", skip this clear zone";
    return true;
  }
  common::SLPoint sl_point;
  sl_point.set_s(clear_zone_overlap.start_s);
  sl_point.set_l(0.0);
  common::math::Vec2d start_xy;
  const auto& reference_line = reference_line_info_->reference_line();
  if (!reference_line.SLToXY(sl_point, &start_xy)) {
    AERROR << "Failed to get xy from sl: " << sl_point.DebugString();
    return false;
  }
  double left_lane_width = 0.0;
  double right_lane_width = 0.0;
  if (!reference_line.GetLaneWidth(clear_zone_overlap.start_s, &left_lane_width,
                                   &right_lane_width)) {
    AERROR << "Failed to get lane width at s = " << clear_zone_overlap.start_s;
    return false;
  }
  common::math::Vec2d end_xy;
  sl_point.set_s(clear_zone_overlap.end_s);
  if (!reference_line_info_->reference_line().SLToXY(sl_point, &end_xy)) {
    AERROR << "Failed to get xy from sl: " << sl_point.DebugString();
    return false;
  }
  common::math::Box2d clear_zone_box{
      common::math::LineSegment2d(start_xy, end_xy),
      left_lane_width + right_lane_width};
  const auto obstacle_id =
      FLAGS_clear_zone_virtual_object_id_prefix + clear_zone_overlap.object_id;
  auto* obstacle =
      frame_->AddStaticVirtualObstacle(obstacle_id, clear_zone_box);
  if (!obstacle) {
    AERROR << "Failed to create obstacle " << obstacle_id << " in frame";
    return false;
  }
  auto* path_obstacle = reference_line_info_->AddObstacle(obstacle);
  if (!path_obstacle) {
    AERROR << "Failed to create path_obstacle for " << obstacle_id;
    return false;
  }
  path_obstacle->SetStBoundaryType(StBoundary::BoundaryType::KEEP_CLEAR);
  return true;
}

}  // namespace planning
}  // namespace apollo
