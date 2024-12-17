/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/planning_interface_base/task_base/common/path_util/path_assessment_decider_util.h"

#include <algorithm>
#include <utility>

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/planning/planning_interface_base/task_base/common/path_util/path_bounds_decider_util.h"

namespace apollo {
namespace planning {

using apollo::common::math::Box2d;
using apollo::common::math::Polygon2d;
using apollo::common::math::Vec2d;

bool PathAssessmentDeciderUtil::IsValidRegularPath(
    const ReferenceLineInfo& reference_line_info, const PathData& path_data) {
  // Basic sanity checks.
  if (path_data.Empty()) {
    AERROR << path_data.path_label() << ": path data is empty.";
    return false;
  }
  // Check if the path is greatly off the reference line.
  if (IsGreatlyOffReferenceLine(path_data)) {
    AERROR << path_data.path_label() << ": ADC is greatly off reference line.";
    return false;
  }
  // Check if the path is greatly off the road.
  if (IsGreatlyOffRoad(reference_line_info, path_data)) {
    AERROR << path_data.path_label() << ": ADC is greatly off road.";
    return false;
  }
  // Check if there is any collision.
  // if (IsCollidingWithStaticObstacles(reference_line_info, path_data)) {
  //   AINFO << path_data.path_label() << ": ADC has collision.";
  //   return false;
  // }

  if (IsStopOnReverseNeighborLane(reference_line_info, path_data)) {
    AERROR << path_data.path_label() << ": stop at reverse neighbor lane";
    return false;
  }

  return true;
}

bool PathAssessmentDeciderUtil::IsGreatlyOffReferenceLine(
    const PathData& path_data) {
  static constexpr double kOffReferenceLineThreshold = 20.0;
  const auto& frenet_path = path_data.frenet_frame_path();
  for (const auto& frenet_path_point : frenet_path) {
    if (std::fabs(frenet_path_point.l()) > kOffReferenceLineThreshold) {
      AINFO << "Greatly off reference line at s = " << frenet_path_point.s()
            << ", with l = " << frenet_path_point.l();
      return true;
    }
  }
  return false;
}

bool PathAssessmentDeciderUtil::IsGreatlyOffRoad(
    const ReferenceLineInfo& reference_line_info, const PathData& path_data) {
  static constexpr double kOffRoadThreshold = 10.0;
  const auto& frenet_path = path_data.frenet_frame_path();
  for (const auto& frenet_path_point : frenet_path) {
    double road_left_width = 0.0;
    double road_right_width = 0.0;
    if (reference_line_info.reference_line().GetRoadWidth(
            frenet_path_point.s(), &road_left_width, &road_right_width)) {
      if (frenet_path_point.l() > road_left_width + kOffRoadThreshold ||
          frenet_path_point.l() < -road_right_width - kOffRoadThreshold) {
        AINFO << "Greatly off-road at s = " << frenet_path_point.s()
              << ", with l = " << frenet_path_point.l();
        return true;
      }
    }
  }
  return false;
}

bool PathAssessmentDeciderUtil::IsCollidingWithStaticObstacles(
    const ReferenceLineInfo& reference_line_info, const PathData& path_data) {
  // Get all obstacles and convert them into frenet-frame polygons.
  std::vector<const Obstacle*> obstacles;
  const auto& indexed_obstacles =
      reference_line_info.path_decision().obstacles();
  const auto& vehicle_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  double front_edge_to_center = vehicle_param.front_edge_to_center();
  double back_edge_to_center = vehicle_param.back_edge_to_center();
  double path_point_lateral_buffer =
      std::max(vehicle_param.width() / 2.0, vehicle_param.length() / 2.0);
  for (const auto* obstacle : indexed_obstacles.Items()) {
    // Filter out unrelated obstacles.
    if (!PathBoundsDeciderUtil::IsWithinPathDeciderScopeObstacle(*obstacle)) {
      continue;
    }
    // Ignore too small obstacles.
    const auto& obstacle_sl = obstacle->PerceptionSLBoundary();
    if ((obstacle_sl.end_s() - obstacle_sl.start_s()) *
            (obstacle_sl.end_l() - obstacle_sl.start_l()) <
        kMinObstacleArea) {
      continue;
    }
    obstacles.push_back(obstacle);
  }
  // Go through all the four corner points at every path pt, check collision.
  const auto& frenet_path = path_data.frenet_frame_path();
  for (size_t i = 0; i < path_data.discretized_path().size(); ++i) {
    // Skip the point after end point.
    if (path_data.frenet_frame_path().back().s() -
            path_data.frenet_frame_path()[i].s() <
        (FLAGS_num_extra_tail_bound_point + 1) *
                FLAGS_path_bounds_decider_resolution +
            vehicle_param.length()) {
      break;
    }
    double path_point_start_s = frenet_path[i].s() - back_edge_to_center;
    double path_point_end_s = frenet_path[i].s() + front_edge_to_center;
    double path_point_start_l = frenet_path[i].l() - path_point_lateral_buffer;
    double path_point_end_l = frenet_path[i].l() + path_point_lateral_buffer;
    // Check the points near the obstacles
    for (const auto* obstacle : obstacles) {
      const auto& obstacle_sl = obstacle->PerceptionSLBoundary();
      // Filter the path points by s range.
      if (obstacle_sl.start_s() > path_point_end_s ||
          obstacle_sl.end_s() < path_point_start_s) {
        continue;
      }
      if (obstacle_sl.start_l() > path_point_end_l ||
          obstacle_sl.end_l() < path_point_start_l) {
        continue;
      }
      const auto& path_point = path_data.discretized_path()[i];
      const auto& vehicle_box =
          common::VehicleConfigHelper::Instance()->GetBoundingBox(path_point);
      const std::vector<Vec2d>& ABCDpoints = vehicle_box.GetAllCorners();
      const common::math::Polygon2d& obstacle_polygon =
          obstacle->PerceptionPolygon();
      for (const auto& corner_point : ABCDpoints) {
        if (obstacle_polygon.IsPointIn(corner_point)) {
          AERROR << "ADC is colliding with obstacle at path s = "
                 << path_point.s() << ", with obstacle " << obstacle->Id();
          return true;
        }
      }
    }
  }
  return false;
}

bool PathAssessmentDeciderUtil::IsStopOnReverseNeighborLane(
    const ReferenceLineInfo& reference_line_info, const PathData& path_data) {
  if (path_data.path_label().find("left") == std::string::npos &&
      path_data.path_label().find("right") == std::string::npos) {
    return false;
  }

  std::vector<common::SLPoint> all_stop_point_sl =
      reference_line_info.GetAllStopDecisionSLPoint();
  if (all_stop_point_sl.empty()) {
    return false;
  }

  double check_s = 0.0;
  // filter out sidepass stop fence
  static constexpr double kLookForwardBuffer = 5.0;
  const double adc_end_s = reference_line_info.AdcSlBoundary().end_s();
  for (const auto& stop_point_sl : all_stop_point_sl) {
    if (stop_point_sl.s() - adc_end_s < kLookForwardBuffer) {
      continue;
    }
    check_s = stop_point_sl.s();
    break;
  }
  if (check_s <= 0.0) {
    return false;
  }

  double lane_left_width = 0.0;
  double lane_right_width = 0.0;
  if (!reference_line_info.reference_line().GetLaneWidth(
          check_s, &lane_left_width, &lane_right_width)) {
    return false;
  }

  static constexpr double kSDelta = 0.3;
  common::SLPoint path_point_sl;
  for (const auto& frenet_path_point : path_data.frenet_frame_path()) {
    if (std::fabs(frenet_path_point.s() - check_s) < kSDelta) {
      path_point_sl.set_s(frenet_path_point.s());
      path_point_sl.set_l(frenet_path_point.l());
    }
  }
  ADEBUG << "path_point_sl[" << path_point_sl.s() << ", " << path_point_sl.l()
         << "] lane_left_width[" << lane_left_width << "] lane_right_width["
         << lane_right_width << "]";

  hdmap::Id neighbor_lane_id;
  double neighbor_lane_width = 0.0;
  if (path_data.path_label().find("left") != std::string::npos &&
      path_point_sl.l() > lane_left_width) {
    if (reference_line_info.GetNeighborLaneInfo(
            ReferenceLineInfo::LaneType::LeftForward, path_point_sl.s(),
            &neighbor_lane_id, &neighbor_lane_width)) {
      AINFO << "stop path point at LeftForward neighbor lane["
            << neighbor_lane_id.id() << "]";
      return false;
    } else {
      AINFO << "stop path point at LeftReverse neighbor lane";
      return true;
    }
  } else if (path_data.path_label().find("right") != std::string::npos &&
             path_point_sl.l() < -lane_right_width) {
    if (reference_line_info.GetNeighborLaneInfo(
            ReferenceLineInfo::LaneType::RightForward, path_point_sl.s(),
            &neighbor_lane_id, &neighbor_lane_width)) {
      AINFO << "stop path point at RightForward neighbor lane["
            << neighbor_lane_id.id() << "]";
      return false;
    } else {
      AINFO << "stop path point at RightReverse neighbor lane";
      return true;
    }
  }
  return false;
}

void PathAssessmentDeciderUtil::InitPathPointDecision(
    const PathData& path_data, const PathData::PathPointType type,
    std::vector<PathPointDecision>* const path_point_decision) {
  // Sanity checks.
  CHECK_NOTNULL(path_point_decision);
  path_point_decision->clear();

  // Go through every path point in path data, and initialize a
  // corresponding path point decision.
  for (const auto& frenet_path_point : path_data.frenet_frame_path()) {
    path_point_decision->emplace_back(frenet_path_point.s(), type,
                                      std::numeric_limits<double>::max());
  }
}

void PathAssessmentDeciderUtil::TrimTailingOutLanePoints(
    PathData* const path_data) {
  // Don't trim self-lane path or fallback path.
  if (path_data->path_label().find("fallback") != std::string::npos ||
      path_data->path_label().find("self") != std::string::npos) {
    return;
  }

  // Trim.
  AINFO << "Trimming " << path_data->path_label();
  auto frenet_path = path_data->frenet_frame_path();
  auto path_point_decision = path_data->path_point_decision_guide();
  while (!path_point_decision.empty() &&
         std::get<1>(path_point_decision.back()) !=
             PathData::PathPointType::IN_LANE) {
    if (std::get<1>(path_point_decision.back()) ==
        PathData::PathPointType::OUT_ON_FORWARD_LANE) {
      AINFO << "Trimming out forward lane point";
    } else if (std::get<1>(path_point_decision.back()) ==
               PathData::PathPointType::OUT_ON_REVERSE_LANE) {
      AINFO << "Trimming out reverse lane point";
    } else {
      AINFO << "Trimming unknown lane point";
    }
    frenet_path.pop_back();
    path_point_decision.pop_back();
  }
  path_data->SetFrenetPath(std::move(frenet_path));
  path_data->SetPathPointDecisionGuide(std::move(path_point_decision));
  AINFO << "After TrimTailingOutLanePoints: FrenetPath size: "
        << path_data->frenet_frame_path().size();
}

}  // namespace planning
}  // namespace apollo
