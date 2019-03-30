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

#include "modules/planning/tasks/deciders/path_assessment_decider.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/tasks/deciders/path_decider_obstacle_utils.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::VehicleConfigHelper;
using apollo::common::math::Polygon2d;
using apollo::common::math::Vec2d;
using apollo::hdmap::HDMapUtil;

// PointDecision contains (s, PathPointType, distance to closest obstacle).
using PathPointDecision = std::tuple<double, PathData::PathPointType, double>;

PathAssessmentDecider::PathAssessmentDecider(const TaskConfig& config)
    : Decider(config) {}

Status PathAssessmentDecider::Process(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  // Sanity checks.
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  // Check the validity of paths (the optimization output).
  // 1. Check the validity of regular and fallback paths.
  PathData* regular_path_data = reference_line_info->mutable_path_data();
  PathData* fallback_path_data =
      reference_line_info->mutable_fallback_path_data();
  bool is_valid_regular_path =
      IsValidRegularPath(*reference_line_info, *regular_path_data);
  bool is_valid_fallback_path =
      IsValidFallbackPath(*reference_line_info, *fallback_path_data);
  // 2. If neither is valid, use the reference_line as the ultimate fallback.
  if (!is_valid_regular_path && !is_valid_fallback_path) {
    reference_line_info->SetFeasiblePathData(
        ReferenceLineInfo::PathDataType::REFERENCE_LINE_PATH);
    const std::string msg = "Neither regular nor fallback path is valid.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // Analyze and add important info for speed decider to use.

  return Status::OK();
}

bool PathAssessmentDecider::IsValidRegularPath(
    const ReferenceLineInfo& reference_line_info, const PathData& path_data) {
  // Check if the path is greatly off the reference line.
  if (IsGreatlyOffReferenceLine(path_data)) {
    return false;
  }
  // Check if the path is greatly off the road.
  if (IsGreatlyOffRoad(reference_line_info, path_data)) {
    return false;
  }
  // Check if there is any collision.
  if (IsCollidingWithStaticObstacles(reference_line_info, path_data)) {
    return false;
  }
  return true;
}

bool PathAssessmentDecider::IsValidFallbackPath(
    const ReferenceLineInfo& reference_line_info, const PathData& path_data) {
  // Check if the path is greatly off the reference line.
  if (IsGreatlyOffReferenceLine(path_data)) {
    return false;
  }
  // Check if the path is greatly off the road.
  if (IsGreatlyOffRoad(reference_line_info, path_data)) {
    return false;
  }
  return true;
}

void PathAssessmentDecider::SetPathInfo(
    const ReferenceLineInfo& reference_line_info, PathData* const path_data) {
  // Go through every path_point, and label its:
  //  - in-lane/out-of-lane info
  //  - distance to the closest obstacle.
  std::vector<PathPointDecision> path_decision;
  InitPathPointDecision(*path_data, &path_decision);
  SetPathPointType(reference_line_info, *path_data, &path_decision);
  SetObstacleDistance(reference_line_info, *path_data, &path_decision);

  path_data->SetPathPointDecisionGuide(path_decision);
}

bool PathAssessmentDecider::IsGreatlyOffReferenceLine(
    const PathData& path_data) {
  constexpr double kOffReferenceLineThreshold = 20.0;
  auto frenet_path = path_data.frenet_frame_path();
  for (const auto& frenet_path_point : frenet_path) {
    if (std::fabs(frenet_path_point.l()) > kOffReferenceLineThreshold) {
      return true;
    }
  }
  return false;
}

bool PathAssessmentDecider::IsGreatlyOffRoad(
    const ReferenceLineInfo& reference_line_info, const PathData& path_data) {
  constexpr double kOffRoadThreshold = 10.0;
  auto frenet_path = path_data.frenet_frame_path();
  for (const auto& frenet_path_point : frenet_path) {
    double road_left_width = 0.0;
    double road_right_width = 0.0;
    if (reference_line_info.reference_line().GetRoadWidth(
            frenet_path_point.s(), &road_left_width, &road_right_width)) {
      if (frenet_path_point.l() > road_left_width + kOffRoadThreshold ||
          frenet_path_point.l() < -road_right_width - kOffRoadThreshold) {
        return true;
      }
    }
  }
  return false;
}

bool PathAssessmentDecider::IsCollidingWithStaticObstacles(
    const ReferenceLineInfo& reference_line_info, const PathData& path_data) {
  // Get all obstacles and convert them into frenet-frame polygons.
  std::vector<Polygon2d> obstacle_polygons;
  auto indexed_obstacles = reference_line_info.path_decision().obstacles();
  for (const auto* obstacle : indexed_obstacles.Items()) {
    // Filter out unrelated obstacles.
    if (!IsWithinPathDeciderScopeObstacle(*obstacle)) {
      continue;
    }
    // Convert into polygon and save it.
    const auto obstacle_sl = obstacle->PerceptionSLBoundary();
    obstacle_polygons.push_back(
        Polygon2d({Vec2d(obstacle_sl.start_s(), obstacle_sl.start_l()),
                   Vec2d(obstacle_sl.start_s(), obstacle_sl.end_l()),
                   Vec2d(obstacle_sl.end_s(), obstacle_sl.end_l()),
                   Vec2d(obstacle_sl.end_s(), obstacle_sl.start_l())}));
  }

  // Go through all the four corner points at every path pt, check collision.
  for (const auto& path_point : path_data.discretized_path()) {
    // Get the four corner points ABCD of ADC at every path point.
    const auto& vehicle_box =
        common::VehicleConfigHelper::Instance()->GetBoundingBox(path_point);
    std::vector<Vec2d> ABCDpoints = vehicle_box.GetAllCorners();
    for (const auto& corner_point : ABCDpoints) {
      // For each corner point, project it onto reference_line
      common::SLPoint curr_point_sl;
      if (!reference_line_info.reference_line().XYToSL(corner_point,
                                                       &curr_point_sl)) {
        AERROR << "Failed to get the projection from point onto "
                  "reference_line";
        return true;
      }
      auto curr_point = Vec2d(curr_point_sl.s(), curr_point_sl.l());
      // Check if it's in any polygon of other static obstacles.
      for (const auto& obstacle_polygon : obstacle_polygons) {
        if (obstacle_polygon.IsPointIn(curr_point)) {
          return true;
        }
      }
    }
  }

  return false;
}

void PathAssessmentDecider::InitPathPointDecision(
    const PathData& path_data,
    std::vector<PathPointDecision>* const path_decision) {
  // Sanity checks.
  CHECK_NOTNULL(path_decision);
  path_decision->clear();

  // Go through every path point in path data, and initialize a
  // corresponding path point decision.
  for (const auto& frenet_path_point : path_data.frenet_frame_path()) {
    path_decision->emplace_back(frenet_path_point.s(),
                                PathData::PathPointType::UNKNOWN,
                                std::numeric_limits<double>::max());
  }
}

void PathAssessmentDecider::SetPathPointType(
    const ReferenceLineInfo& reference_line_info, const PathData& path_data,
    std::vector<PathPointDecision>* const path_decision) {
  // Sanity checks.
  CHECK_NOTNULL(path_decision);

  // Go through every path_point, and add in-lane/out-of-lane info.
  const auto& frenet_path = path_data.frenet_frame_path();
  const auto& discrete_path = path_data.discretized_path();
  double adc_half_width =
      VehicleConfigHelper::GetConfig().vehicle_param().width() / 2.0;
  for (size_t i = 0; i < frenet_path.size(); ++i) {
    const auto& frenet_path_point = frenet_path[i];
    const auto& discrete_path_point = discrete_path[i];
    double lane_left_width = 0.0;
    double lane_right_width = 0.0;
    if (reference_line_info.reference_line().GetLaneWidth(
            frenet_path_point.s(), &lane_left_width, &lane_right_width)) {
      if (frenet_path_point.l() > lane_left_width ||
          frenet_path_point.l() < -lane_right_width) {
        // The path point is out of the reference_line's lane.
        // To be conservative, by default treat it as reverse lane.
        std::get<1>((*path_decision)[i]) =
            PathData::PathPointType::OUT_ON_REVERSE_LANE;
        // Only when the lanes that contain this path point are all
        // forward lanes and none is reverse lane, then treat this
        // path point as OUT_ON_FORWARD_LANE.
        std::vector<hdmap::LaneInfoConstPtr> forward_lanes;
        std::vector<hdmap::LaneInfoConstPtr> reverse_lanes;
        if (HDMapUtil::BaseMapPtr()->GetLanesWithHeading(
                common::util::MakePointENU(discrete_path_point.x(),
                                           discrete_path_point.y(), 0.0),
                adc_half_width, discrete_path_point.theta(), M_PI / 2.0,
                &forward_lanes) == 0 &&
            HDMapUtil::BaseMapPtr()->GetLanesWithHeading(
                common::util::MakePointENU(discrete_path_point.x(),
                                           discrete_path_point.y(), 0.0),
                adc_half_width, discrete_path_point.theta() - M_PI, M_PI / 2.0,
                &reverse_lanes) == 0) {
          if (!forward_lanes.empty() && reverse_lanes.empty()) {
            std::get<1>((*path_decision)[i]) =
                PathData::PathPointType::OUT_ON_FORWARD_LANE;
          }
        }
      } else {
        // The path point is within the reference_line's lane.
        std::get<1>((*path_decision)[i]) = PathData::PathPointType::IN_LANE;
      }
    }
  }
}

void PathAssessmentDecider::SetObstacleDistance(
    const ReferenceLineInfo& reference_line_info, const PathData& path_data,
    std::vector<PathPointDecision>* const path_decision) {
  // Sanity checks
  CHECK_NOTNULL(path_decision);

  // Get all obstacles and convert them into frenet-frame polygons.
  std::vector<Polygon2d> obstacle_polygons;
  auto indexed_obstacles = reference_line_info.path_decision().obstacles();
  for (const auto* obstacle : indexed_obstacles.Items()) {
    // Filter out unrelated obstacles.
    if (!IsWithinPathDeciderScopeObstacle(*obstacle)) {
      continue;
    }
    // Convert into polygon and save it.
    const auto obstacle_box = obstacle->PerceptionBoundingBox();
    obstacle_polygons.push_back(Polygon2d(obstacle_box));
  }

  // Go through every path point, update closest obstacle info.
  const auto& discrete_path = path_data.discretized_path();
  for (size_t i = 0; i < discrete_path.size(); ++i) {
    const auto& path_point = discrete_path[i];
    // Get the bounding box of the vehicle at that point.
    const auto& vehicle_box =
        common::VehicleConfigHelper::Instance()->GetBoundingBox(path_point);
    // Go through all the obstacle polygons, and update the min distance.
    double min_distance_to_obstacles = std::numeric_limits<double>::max();
    for (const auto& obstacle_polygon : obstacle_polygons) {
      double distance_to_vehicle = obstacle_polygon.DistanceTo(vehicle_box);
      min_distance_to_obstacles =
          std::min(min_distance_to_obstacles, distance_to_vehicle);
    }
    std::get<2>((*path_decision)[i]) = min_distance_to_obstacles;
  }
}

}  // namespace planning
}  // namespace apollo
