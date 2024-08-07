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

#include "modules/planning/planning_interface_base/task_base/common/path_util/path_bounds_decider_util.h"

#include <algorithm>
#include <functional>
#include <limits>
#include <set>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/linear_interpolation.h"
#include "modules/common/util/util.h"
#include "modules/planning/planning_base/common/sl_polygon.h"
#include "modules/planning/planning_base/common/util/util.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::VehicleConfigHelper;

bool PathBoundsDeciderUtil::InitPathBoundary(
    const ReferenceLineInfo& reference_line_info,
    PathBoundary* const path_bound, SLState init_sl_state) {
  // Sanity checks.
  CHECK_NOTNULL(path_bound);
  path_bound->clear();
  const auto& reference_line = reference_line_info.reference_line();
  path_bound->set_delta_s(FLAGS_path_bounds_decider_resolution);

  const auto& vehicle_config =
      common::VehicleConfigHelper::Instance()->GetConfig();
  const double ego_front_to_center =
      vehicle_config.vehicle_param().front_edge_to_center();

  for (double curr_s = init_sl_state.first[0];
       curr_s < std::fmin(init_sl_state.first[0] +
                              std::fmax(FLAGS_path_bounds_horizon,
                                        reference_line_info.GetCruiseSpeed() *
                                            FLAGS_trajectory_time_length),
                          reference_line.Length() - ego_front_to_center);
       curr_s += FLAGS_path_bounds_decider_resolution) {
    path_bound->emplace_back(curr_s, std::numeric_limits<double>::lowest(),
                             std::numeric_limits<double>::max());
  }

  // Return.
  if (path_bound->empty()) {
    ADEBUG << "Empty path boundary in InitPathBoundary";
    return false;
  }
  return true;
}

void PathBoundsDeciderUtil::GetStartPoint(
    common::TrajectoryPoint planning_start_point,
    const ReferenceLine& reference_line, SLState* init_sl_state) {
  if (FLAGS_use_front_axe_center_in_path_planning) {
    planning_start_point =
        InferFrontAxeCenterFromRearAxeCenter(planning_start_point);
  }
  AINFO << std::fixed << "Plan at the starting point: x = "
        << planning_start_point.path_point().x()
        << ", y = " << planning_start_point.path_point().y()
        << ", and angle = " << planning_start_point.path_point().theta();

  // Initialize some private variables.
  // ADC s/l info.
  *init_sl_state = reference_line.ToFrenetFrame(planning_start_point);
}

double PathBoundsDeciderUtil::GetADCLaneWidth(
    const ReferenceLine& reference_line, const double adc_s) {
  double lane_left_width = 0.0;
  double lane_right_width = 0.0;
  if (!reference_line.GetLaneWidth(adc_s, &lane_left_width,
                                   &lane_right_width)) {
    constexpr double kDefaultLaneWidth = 5.0;
    AWARN << "Failed to get lane width at planning start point.";
    return kDefaultLaneWidth;
  } else {
    return lane_left_width + lane_right_width;
  }
}

bool PathBoundsDeciderUtil::UpdatePathBoundaryWithBuffer(
    double left_bound, double right_bound, BoundType left_type,
    BoundType right_type, std::string left_id, std::string right_id,
    PathBoundPoint* const bound_point) {
  if (!UpdateLeftPathBoundaryWithBuffer(left_bound, left_type, left_id,
                                        bound_point)) {
    return false;
  }
  if (!UpdateRightPathBoundaryWithBuffer(right_bound, right_type, right_id,
                                         bound_point)) {
    return false;
  }
  return true;
}

bool PathBoundsDeciderUtil::UpdateLeftPathBoundaryWithBuffer(
    double left_bound, BoundType left_type, std::string left_id,
    PathBoundPoint* const bound_point) {
  double adc_half_width =
      VehicleConfigHelper::GetConfig().vehicle_param().width() / 2.0;
  left_bound = left_bound - adc_half_width;
  PathBoundPoint new_point = *bound_point;
  if (new_point.l_upper.l > left_bound) {
    new_point.l_upper.l = left_bound;
    new_point.l_upper.type = left_type;
    new_point.l_upper.id = left_id;
  }
  // Check if ADC is blocked.
  // If blocked, don't update anything, return false.
  if (new_point.l_lower.l > new_point.l_upper.l) {
    ADEBUG << "Path is blocked at" << new_point.l_lower.l << " "
           << new_point.l_upper.l;
    return false;
  }
  // Otherwise, update path_boundaries and center_line; then return true.
  *bound_point = new_point;
  return true;
}
bool PathBoundsDeciderUtil::UpdateRightPathBoundaryWithBuffer(
    double right_bound, BoundType right_type, std::string right_id,
    PathBoundPoint* const bound_point) {
  double adc_half_width =
      VehicleConfigHelper::GetConfig().vehicle_param().width() / 2.0;
  right_bound = right_bound + adc_half_width;
  PathBoundPoint new_point = *bound_point;
  if (new_point.l_lower.l < right_bound) {
    new_point.l_lower.l = right_bound;
    new_point.l_lower.type = right_type;
    new_point.l_lower.id = right_id;
  }
  // Check if ADC is blocked.
  // If blocked, don't update anything, return false.
  if (new_point.l_lower.l > new_point.l_upper.l) {
    ADEBUG << "Path is blocked at";
    return false;
  }
  // Otherwise, update path_boundaries and center_line; then return true.
  *bound_point = new_point;
  return true;
}

void PathBoundsDeciderUtil::TrimPathBounds(
    const int path_blocked_idx, PathBoundary* const path_boundaries) {
  if (path_blocked_idx != -1) {
    if (path_blocked_idx == 0) {
      ADEBUG << "Completely blocked. Cannot move at all.";
    }
    double front_edge_to_center =
        VehicleConfigHelper::GetConfig().vehicle_param().front_edge_to_center();
    double trimmed_s =
        path_boundaries->at(path_blocked_idx).s - front_edge_to_center;
    AINFO << "Trimmed from " << path_boundaries->back().s << " to "
          << path_boundaries->at(path_blocked_idx - 1).s;
    while (path_boundaries->size() > 1 &&
           path_boundaries->back().s > trimmed_s) {
      path_boundaries->pop_back();
    }
  }
}

void PathBoundsDeciderUtil::GetSLPolygons(
    const ReferenceLineInfo& reference_line_info,
    std::vector<SLPolygon>* polygons, const SLState& init_sl_state) {
  polygons->clear();
  auto obstacles = reference_line_info.path_decision().obstacles();
  const double adc_back_edge_s = reference_line_info.AdcSlBoundary().start_s();
  for (const auto* obstacle : obstacles.Items()) {
    if (!IsWithinPathDeciderScopeObstacle(*obstacle)) {
      continue;
    }
    auto xy_poly = obstacle->PerceptionPolygon();

    // if (obstacle->PerceptionSLBoundary().end_s() < init_sl_state.first[0]) {
    //     continue;
    // }
    if (obstacle->PerceptionSLBoundary().end_s() < adc_back_edge_s) {
      continue;
    }
    const auto obstacle_sl = obstacle->PerceptionSLBoundary();
    polygons->emplace_back(obstacle_sl, obstacle->Id());
  }
  sort(polygons->begin(), polygons->end(),
       [](const SLPolygon& a, const SLPolygon& b) {
         return a.MinS() < b.MinS();
       });
}

bool PathBoundsDeciderUtil::UpdatePathBoundaryBySLPolygon(
    PathBoundary* path_boundary, std::vector<SLPolygon>* sl_polygon,
    std::vector<double>* center_l, std::string* blocked_id,
    double* narrowest_width) {
  if (center_l->empty()) {
    center_l->push_back(
        (path_boundary->front().l_lower.l + path_boundary->front().l_upper.l) /
        2.0);
  }
  *narrowest_width =
      path_boundary->front().l_upper.l - path_boundary->front().l_lower.l;
  size_t nudge_check_count = size_t(FLAGS_max_nudge_check_distance /
                                    FLAGS_path_bounds_decider_resolution);
  double last_max_nudge_l = center_l->front();

  for (size_t i = 1; i < path_boundary->size(); ++i) {
    auto& left_bound = path_boundary->at(i).l_upper;
    auto& right_bound = path_boundary->at(i).l_lower;
    double default_width = right_bound.l - left_bound.l;
    auto begin_it =
        center_l->end() - std::min(nudge_check_count, center_l->size());
    last_max_nudge_l = *std::max_element(
        begin_it, center_l->end(),
        [](double a, double b) { return std::fabs(a) < std::fabs(b); });
    AINFO << "last max nudge l: " << last_max_nudge_l;
    for (size_t j = 0; j < sl_polygon->size(); j++) {
      // double min_s = sl_polygon->at(j).MinS() -
      // FLAGS_path_bounds_decider_resolution; double max_s =
      // sl_polygon->at(j).MaxS() + FLAGS_path_bounds_decider_resolution;
      double min_s = sl_polygon->at(j).MinS();
      double max_s =
          sl_polygon->at(j).MaxS() + FLAGS_obstacle_lon_end_buffer_park;
      if (max_s - min_s < FLAGS_path_bounds_decider_resolution) {
        max_s += FLAGS_path_bounds_decider_resolution;
        min_s -= FLAGS_path_bounds_decider_resolution;
      }
      if (max_s < path_boundary->at(i).s) {
        continue;
      }
      if (min_s > path_boundary->at(i).s) {
        break;
      }
      double l_lower = std::max<double>(
          sl_polygon->at(j).GetRightBoundaryByS(path_boundary->at(i).s),
          path_boundary->at(i).l_lower.l);
      double l_upper = std::min<double>(
          sl_polygon->at(j).GetLeftBoundaryByS(path_boundary->at(i).s),
          path_boundary->at(i).l_upper.l);
      if (sl_polygon->at(j).NudgeInfo() == SLPolygon::UNDEFINED) {
        AINFO << "last_max_nudge_l: " << last_max_nudge_l
              << ", obs l: " << l_lower << ", " << l_upper;
        if (last_max_nudge_l < (l_lower + l_upper) / 2) {
          sl_polygon->at(j).SetNudgeInfo(SLPolygon::RIGHT_NUDGE);
          AINFO << sl_polygon->at(j).id() << " right nudge";
        } else {
          sl_polygon->at(j).SetNudgeInfo(SLPolygon::LEFT_NUDGE);
          AINFO << sl_polygon->at(j).id() << " left nudge";
        }
      } else {
        AINFO << "last_max_nudge_l: " << last_max_nudge_l
              << ", obs l: " << l_lower << ", " << l_upper
              << ", nudge info: " << sl_polygon->at(j).NudgeInfo();
      }
      if (sl_polygon->at(j).NudgeInfo() == SLPolygon::RIGHT_NUDGE) {
        // right nudge
        l_lower -= GetBufferBetweenADCCenterAndEdge();
        if (l_lower <= right_bound.l) {
          // boundary is blocked
          *blocked_id = sl_polygon->at(j).id();
          AINFO << "blocked at " << *blocked_id << ", "
                << path_boundary->at(i).s << ", l_lower: " << l_lower
                << ", l_upper: " << l_upper;
          sl_polygon->at(j).SetNudgeInfo(SLPolygon::BLOCKED);
          break;
        }
        if (l_lower < left_bound.l) {
          AINFO << "update left_bound [s, l]: [" << path_boundary->at(i).s
                << ", " << l_lower << "]";
          left_bound.l = l_lower;
          left_bound.type = BoundType::OBSTACLE;
          left_bound.id = sl_polygon->at(j).id();
          *narrowest_width =
              std::min(*narrowest_width, left_bound.l - right_bound.l);
        }
      } else {
        // left nudge
        l_upper += GetBufferBetweenADCCenterAndEdge();
        if (l_upper >= left_bound.l) {
          // boundary is blocked
          *blocked_id = sl_polygon->at(j).id();
          AINFO << "blocked at " << *blocked_id << ", "
                << path_boundary->at(i).s << ", l_lower: " << l_lower
                << ", l_upper: " << l_upper;
          sl_polygon->at(j).SetNudgeInfo(SLPolygon::BLOCKED);
          break;
        }
        if (l_upper > right_bound.l) {
          AINFO << "update right_bound [s, l]: [" << path_boundary->at(i).s
                << ", " << l_upper << "]";
          right_bound.l = l_upper;
          right_bound.type = BoundType::OBSTACLE;
          right_bound.id = sl_polygon->at(j).id();
          *narrowest_width =
              std::min(*narrowest_width, left_bound.l - right_bound.l);
        }
      }
    }
    // if blocked, trim path
    if (!blocked_id->empty()) {
      TrimPathBounds(i, path_boundary);
      *narrowest_width = default_width;
      return false;
    }
    center_l->push_back((left_bound.l + right_bound.l) / 2.0);
    AINFO << "update center_l: " << (left_bound.l + right_bound.l) / 2.0;
  }
  return true;
}

bool PathBoundsDeciderUtil::AddCornerPoint(
    double s, double l_lower, double l_upper, const PathBoundary& path_boundary,
    InterPolatedPointVec* extra_constraints) {
  size_t left_index = 0;
  size_t right_index = 0;
  double left_weight = 0.0;
  double right_weight = 0.0;
  if (!path_boundary.get_interpolated_s_weight(s, &left_weight, &right_weight,
                                               &left_index, &right_index)) {
    AERROR << "Fail to find extra path bound point in path boundary: " << s
           << ", path boundary start s: " << path_boundary.front().s
           << ", path boundary end s: " << path_boundary.back().s;
    return false;
  }
  if (left_weight < 0.05 || right_weight < 0.05) {
    // filter contraint that near evaulated point
    return false;
  }
  ADEBUG << "corner" << s << "left_weight" << left_weight << "right_weight"
         << right_weight << "left_index" << left_index << "right_index"
         << right_index << "l_lower" << l_lower << "l_upper" << l_upper;
  extra_constraints->emplace_back(left_weight, right_weight, l_lower, l_upper,
                                  left_index, right_index, s);
  return true;
}

void PathBoundsDeciderUtil::AddCornerBounds(
    const std::vector<SLPolygon>& sl_polygons, PathBoundary* path_boundary) {
  std::vector<SLPoint> left_corner_points;
  std::vector<SLPoint> right_corner_points;
  auto* extra_path_bound = path_boundary->mutable_extra_path_bound();
  for (const auto& obs_polygon : sl_polygons) {
    if (obs_polygon.MinS() > path_boundary->back().s) {
      ADEBUG << "obs_polygon.MinS()" << obs_polygon.MinS()
             << "path_boundary->back().s" << path_boundary->back().s;
      break;
    }
    if (obs_polygon.MaxS() < path_boundary->front().s) {
      continue;
    }
    if (obs_polygon.NudgeInfo() == SLPolygon::LEFT_NUDGE) {
      for (auto pt : obs_polygon.LeftBoundary()) {
        right_corner_points.emplace_back(pt);
      }
    } else if (obs_polygon.NudgeInfo() == SLPolygon::RIGHT_NUDGE) {
      for (auto pt : obs_polygon.RightBoundary()) {
        left_corner_points.emplace_back(pt);
      }
    }
  }
  for (const auto corner_point : left_corner_points) {
    double corner_l = corner_point.l() - GetBufferBetweenADCCenterAndEdge();
    double bound_l_upper =
        path_boundary->get_upper_bound_by_s(corner_point.s());
    double bound_l_lower =
        path_boundary->get_lower_bound_by_s(corner_point.s());
    ADEBUG << corner_point.s() << "left corner_l" << corner_l << "bound_l_upper"
           << bound_l_upper << "bound_l_lower" << bound_l_lower;
    if (corner_l < bound_l_upper) {
      AddCornerPoint(corner_point.s(), bound_l_lower, corner_l, *path_boundary,
                     extra_path_bound);
    }
  }
  for (const auto corner_point : right_corner_points) {
    double corner_l = corner_point.l() + GetBufferBetweenADCCenterAndEdge();
    double bound_l_upper =
        path_boundary->get_upper_bound_by_s(corner_point.s());
    double bound_l_lower =
        path_boundary->get_lower_bound_by_s(corner_point.s());
    ADEBUG << corner_point.s() << "right corner_l" << corner_l
           << "bound_l_upper" << bound_l_upper << "bound_l_lower"
           << bound_l_lower;
    if (corner_l > bound_l_lower) {
      AddCornerPoint(corner_point.s(), corner_l, bound_l_upper, *path_boundary,
                     extra_path_bound);
    }
  }
}

bool PathBoundsDeciderUtil::GetBoundaryFromStaticObstacles(
    std::vector<SLPolygon>* const sl_polygons, const SLState& init_sl_state,
    PathBoundary* const path_boundary, std::string* const blocking_obstacle_id,
    double* const narrowest_width) {
  std::vector<double> center_line;
  // center_line.push_back(init_sl_state.second[0]);
  center_line.push_back(0.0);
  UpdatePathBoundaryBySLPolygon(path_boundary, sl_polygons, &center_line,
                                blocking_obstacle_id, narrowest_width);
  PrintCurves print_curve;
  for (const auto& pt : *path_boundary) {
    print_curve.AddPoint("obs_polygon_l_lower", pt.s, pt.l_lower.l);
    print_curve.AddPoint("obs_polygon_l_upper", pt.s, pt.l_upper.l);
  }
  RelaxEgoLateralBoundary(path_boundary, init_sl_state);
  if (FLAGS_enable_corner_constraint) {
    AddCornerBounds(*sl_polygons, path_boundary);
  }
  const auto& extra_path_bound = path_boundary->extra_path_bound();
  for (const auto& pt : extra_path_bound) {
    print_curve.AddPoint("extra_bound", pt.rear_axle_s, pt.lower_bound);
    print_curve.AddPoint("extra_bound", pt.rear_axle_s, pt.upper_bound);
    ADEBUG << "extra_bound" << pt.rear_axle_s << "l_lower" << pt.lower_bound
           << "l_upper" << pt.lower_bound;
  }
  print_curve.PrintToLog();
  return true;
}

double PathBoundsDeciderUtil::GetBufferBetweenADCCenterAndEdge() {
  double adc_half_width =
      VehicleConfigHelper::GetConfig().vehicle_param().width() / 2.0;

  return (adc_half_width + FLAGS_obstacle_lat_buffer);
}

bool PathBoundsDeciderUtil::IsWithinPathDeciderScopeObstacle(
    const Obstacle& obstacle) {
  // Obstacle should be non-virtual.
  if (obstacle.IsVirtual()) {
    return false;
  }
  // Obstacle should not have ignore decision.
  if (obstacle.HasLongitudinalDecision() && obstacle.HasLateralDecision() &&
      obstacle.IsIgnore()) {
    return false;
  }
  // Obstacle should not be moving obstacle.
  if (!obstacle.IsStatic() ||
      obstacle.speed() > FLAGS_static_obstacle_speed_threshold) {
    return false;
  }
  // TODO(jiacheng):
  // Some obstacles are not moving, but only because they are waiting for
  // red light (traffic rule) or because they are blocked by others (social).
  // These obstacles will almost certainly move in the near future and we
  // should not side-pass such obstacles.

  return true;
}

common::TrajectoryPoint
PathBoundsDeciderUtil::InferFrontAxeCenterFromRearAxeCenter(
    const common::TrajectoryPoint& traj_point) {
  double front_to_rear_axe_distance =
      VehicleConfigHelper::GetConfig().vehicle_param().wheel_base();
  common::TrajectoryPoint ret = traj_point;
  ret.mutable_path_point()->set_x(
      traj_point.path_point().x() +
      front_to_rear_axe_distance * std::cos(traj_point.path_point().theta()));
  ret.mutable_path_point()->set_y(
      traj_point.path_point().y() +
      front_to_rear_axe_distance * std::sin(traj_point.path_point().theta()));
  return ret;
}

bool PathBoundsDeciderUtil::GetBoundaryFromSelfLane(
    const ReferenceLineInfo& reference_line_info, const SLState& init_sl_state,
    PathBoundary* const path_bound) {
  // Sanity checks.
  CHECK_NOTNULL(path_bound);
  ACHECK(!path_bound->empty());
  const ReferenceLine& reference_line = reference_line_info.reference_line();
  double adc_lane_width =
      GetADCLaneWidth(reference_line, init_sl_state.first[0]);
  // Go through every point, update the boundary based on lane info and
  // ADC's position.
  double past_lane_left_width = adc_lane_width / 2.0;
  double past_lane_right_width = adc_lane_width / 2.0;
  int path_blocked_idx = -1;
  for (size_t i = 0; i < path_bound->size(); ++i) {
    double curr_s = (*path_bound)[i].s;
    // 1. Get the current lane width at current point.
    double curr_lane_left_width = 0.0;
    double curr_lane_right_width = 0.0;
    double offset_to_lane_center = 0.0;
    if (!reference_line.GetLaneWidth(curr_s, &curr_lane_left_width,
                                     &curr_lane_right_width)) {
      AWARN << "Failed to get lane width at s = " << curr_s;
      curr_lane_left_width = past_lane_left_width;
      curr_lane_right_width = past_lane_right_width;
    } else {
      reference_line.GetOffsetToMap(curr_s, &offset_to_lane_center);
      curr_lane_left_width += offset_to_lane_center;
      curr_lane_right_width -= offset_to_lane_center;
      past_lane_left_width = curr_lane_left_width;
      past_lane_right_width = curr_lane_right_width;
    }

    // 3. Calculate the proper boundary based on lane-width, ADC's position,
    //    and ADC's velocity.
    double offset_to_map = 0.0;
    reference_line.GetOffsetToMap(curr_s, &offset_to_map);

    double curr_left_bound = 0.0;
    double curr_right_bound = 0.0;
    curr_left_bound = curr_lane_left_width - offset_to_map;
    curr_right_bound = -curr_lane_right_width - offset_to_map;
    // 4. Update the boundary.
    if (!UpdatePathBoundaryWithBuffer(curr_left_bound, curr_right_bound,
                                      BoundType::LANE, BoundType::LANE, "", "",
                                      &path_bound->at(i))) {
      path_blocked_idx = static_cast<int>(i);
    }
    if (path_blocked_idx != -1) {
      break;
    }
  }

  PathBoundsDeciderUtil::TrimPathBounds(path_blocked_idx, path_bound);

  return true;
}

bool PathBoundsDeciderUtil::GetBoundaryFromRoad(
    const ReferenceLineInfo& reference_line_info, const SLState& init_sl_state,
    PathBoundary* const path_bound) {
  // Sanity checks.
  CHECK_NOTNULL(path_bound);
  ACHECK(!path_bound->empty());
  const ReferenceLine& reference_line = reference_line_info.reference_line();
  double adc_lane_width =
      GetADCLaneWidth(reference_line, init_sl_state.first[0]);
  // Go through every point, update the boudnary based on the road boundary.
  double past_road_left_width = adc_lane_width / 2.0;
  double past_road_right_width = adc_lane_width / 2.0;
  int path_blocked_idx = -1;
  for (size_t i = 0; i < path_bound->size(); ++i) {
    // 1. Get road boundary.
    double curr_s = (*path_bound)[i].s;
    double curr_road_left_width = 0.0;
    double curr_road_right_width = 0.0;
    if (!reference_line.GetRoadWidth(curr_s, &curr_road_left_width,
                                     &curr_road_right_width)) {
      AWARN << "Failed to get lane width at s = " << curr_s;
      curr_road_left_width = past_road_left_width;
      curr_road_right_width = past_road_right_width;
    }

    past_road_left_width = curr_road_left_width;
    past_road_right_width = curr_road_right_width;

    double curr_left_bound = curr_road_left_width;
    double curr_right_bound = -curr_road_right_width;
    ADEBUG << "At s = " << curr_s
           << ", left road bound = " << curr_road_left_width
           << ", right road bound = " << curr_road_right_width;

    // 2. Update into path_bound.
    if (!UpdatePathBoundaryWithBuffer(curr_left_bound, curr_right_bound,
                                      BoundType::ROAD, BoundType::ROAD, "", "",
                                      &path_bound->at(i))) {
      path_blocked_idx = static_cast<int>(i);
    }
    if (path_blocked_idx != -1) {
      break;
    }
  }
  AINFO << "path_blocked_idx: " << path_blocked_idx;
  TrimPathBounds(path_blocked_idx, path_bound);
  return true;
}

bool PathBoundsDeciderUtil::ExtendBoundaryByADC(
    const ReferenceLineInfo& reference_line_info, const SLState& init_sl_state,
    const double extend_buffer, PathBoundary* const path_bound) {
  double adc_l_to_lane_center = init_sl_state.second[0];
  static constexpr double kMaxLateralAccelerations = 1.5;

  double ADC_speed_buffer = (init_sl_state.second[1] > 0 ? 1.0 : -1.0) *
                            init_sl_state.second[1] * init_sl_state.second[1] /
                            kMaxLateralAccelerations / 2.0;
  double adc_half_width =
      VehicleConfigHelper::GetConfig().vehicle_param().width() / 2.0;
  double left_bound_adc =
      std::fmax(adc_l_to_lane_center, adc_l_to_lane_center + ADC_speed_buffer) +
      adc_half_width + extend_buffer;
  double right_bound_adc =
      std::fmin(adc_l_to_lane_center, adc_l_to_lane_center + ADC_speed_buffer) -
      adc_half_width - extend_buffer;

  static constexpr double kEpsilon = 0.05;
  for (size_t i = 0; i < path_bound->size(); ++i) {
    double road_left_width = std::fabs(left_bound_adc) + kEpsilon;
    double road_right_width = std::fabs(right_bound_adc) + kEpsilon;
    reference_line_info.reference_line().GetRoadWidth(
        (*path_bound)[i].s, &road_left_width, &road_right_width);
    double left_bound_road = road_left_width - adc_half_width;
    double right_bound_road = -road_right_width + adc_half_width;

    if (left_bound_adc > (*path_bound)[i].l_upper.l) {
      (*path_bound)[i].l_upper.l =
          std::max(std::min(left_bound_adc, left_bound_road),
                   (*path_bound)[i].l_upper.l);
      (*path_bound)[i].l_upper.type = BoundType::ADC;
      (*path_bound)[i].l_upper.id = "adc";
    }
    if (right_bound_adc < (*path_bound)[i].l_lower.l) {
      (*path_bound)[i].l_lower.l =
          std::min(std::max(right_bound_adc, right_bound_road),
                   (*path_bound)[i].l_lower.l);
      (*path_bound)[i].l_lower.type = BoundType::ADC;
      (*path_bound)[i].l_lower.id = "adc";
    }
  }
  return true;
}

void PathBoundsDeciderUtil::ConvertBoundarySAxisFromLaneCenterToRefLine(
    const ReferenceLineInfo& reference_line_info,
    PathBoundary* const path_bound) {
  const ReferenceLine& reference_line = reference_line_info.reference_line();
  for (size_t i = 0; i < path_bound->size(); ++i) {
    // 1. Get road boundary.
    double curr_s = (*path_bound)[i].s;
    double refline_offset_to_lane_center = 0.0;
    reference_line.GetOffsetToMap(curr_s, &refline_offset_to_lane_center);
    (*path_bound)[i].l_lower.l -= refline_offset_to_lane_center;
    (*path_bound)[i].l_upper.l -= refline_offset_to_lane_center;
  }
}

int PathBoundsDeciderUtil::IsPointWithinPathBound(
    const ReferenceLineInfo& reference_line_info, const double x,
    const double y, const PathBound& path_bound) {
  common::SLPoint point_sl;
  reference_line_info.reference_line().XYToSL({x, y}, &point_sl);
  if (point_sl.s() > path_bound.back().s ||
      point_sl.s() <
          path_bound.front().s - FLAGS_path_bounds_decider_resolution * 2) {
    ADEBUG << "Longitudinally outside the boundary.";
    return -1;
  }
  int idx_after = 0;
  while (idx_after < static_cast<int>(path_bound.size()) &&
         path_bound[idx_after].s < point_sl.s()) {
    ++idx_after;
  }
  ADEBUG << "The idx_after = " << idx_after;
  ADEBUG << "The boundary is: "
         << "[" << path_bound[idx_after].l_lower.l << ", "
         << path_bound[idx_after].l_upper.l << "].";
  ADEBUG << "The point is at: " << point_sl.l();
  int idx_before = idx_after - 1;
  if (path_bound[idx_before].l_lower.l <= point_sl.l() &&
      path_bound[idx_before].l_upper.l >= point_sl.l() &&
      path_bound[idx_after].l_lower.l <= point_sl.l() &&
      path_bound[idx_after].l_upper.l >= point_sl.l()) {
    return idx_after;
  }
  ADEBUG << "Laterally outside the boundary.";
  return -1;
}

bool PathBoundsDeciderUtil::RelaxEgoLateralBoundary(
    PathBoundary* path_boundary, const SLState& init_sl_state) {
  if (path_boundary->size() < 2) {
    AINFO << "path_boundary size = 0, return.";
    return false;
  }
  const auto& init_pt = path_boundary->at(0);
  double min_radius =
      std::min(FLAGS_relax_ego_radius, common::VehicleConfigHelper::Instance()
                                           ->GetConfig()
                                           .vehicle_param()
                                           .min_turn_radius());
  double init_frenet_heading =
      common::math::Vec2d(1.0, init_sl_state.second[1]).Angle();
  double init_pt_l = init_sl_state.second[0];
  for (size_t i = 1; i < path_boundary->size(); ++i) {
    auto& left_bound = path_boundary->at(i).l_upper;
    auto& right_bound = path_boundary->at(i).l_lower;
    if (left_bound.type == BoundType::OBSTACLE) {
      double protective_restrict = util::left_arc_bound_with_heading(
          path_boundary->at(i).s - init_pt.s, min_radius, init_frenet_heading);
      double left_obs_constraint =
          std::max(left_bound.l, init_pt_l + protective_restrict);
      AINFO << "init_pt_l: " << init_pt_l << ", left_bound: " << left_bound.l
            << ",  diff s: " << path_boundary->at(i).s - init_pt.s
            << ", min_radius: " << min_radius
            << ", init_frenet_heading: " << init_frenet_heading
            << ", protective_restrict: " << protective_restrict
            << ", left_obs_constraint: " << left_obs_constraint;
      left_obs_constraint = std::min(left_bound.l + FLAGS_obstacle_lat_buffer -
                                         FLAGS_ego_front_slack_buffer,
                                     left_obs_constraint);
      AINFO << "left_obs_constraint: " << left_obs_constraint;
      left_bound.l = left_obs_constraint;
    }

    if (right_bound.type == BoundType::OBSTACLE) {
      double protective_restrict = util::right_arc_bound_with_heading(
          path_boundary->at(i).s - init_pt.s, min_radius, init_frenet_heading);
      double right_obs_constraint =
          std::min(right_bound.l, init_pt_l + protective_restrict);
      AINFO << "init_pt_l: " << init_pt_l << ", right_bound: " << right_bound.l
            << ",  diff s: " << path_boundary->at(i).s - init_pt.s
            << ", min_radius: " << min_radius
            << ", init_frenet_heading: " << init_frenet_heading
            << ", protective_restrict: " << protective_restrict
            << ", right_obs_constraint: " << right_obs_constraint;
      right_obs_constraint =
          std::max(right_bound.l - FLAGS_obstacle_lat_buffer +
                       FLAGS_ego_front_slack_buffer,
                   right_obs_constraint);
      AINFO << "right_obs_constraint: " << right_obs_constraint;
      right_bound.l = right_obs_constraint;
    }
  }
  return true;
}

}  // namespace planning
}  // namespace apollo
