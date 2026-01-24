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

/**
 * @file
 **/

#include "modules/planning/planning_base/common/sl_polygon.h"

#include <limits>
#include <string>
#include <utility>

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::VehicleConfigHelper;


using apollo::common::VehicleConfigHelper;

double SLPolygon::GetInterpolatedLFromBoundary(
    const std::vector<SLPoint>& boundary, const double s) {
  ACHECK(!boundary.empty());
  if (s <= boundary.front().s()) {
    return boundary.front().l();
  } else if (s >= boundary.back().s()) {
    return boundary.back().l();
  }
  SLPoint sl_point;
  sl_point.set_s(s);
  sl_point.set_l(0.0);
  auto cmp = [](const SLPoint& sl_point, const double s) {
    return sl_point.s() < s;
  };
  auto iter = std::lower_bound(boundary.begin(), boundary.end(), s, cmp);
  auto last_iter = *(iter - 1);
  return last_iter.l() + (s - last_iter.s()) * (iter->l() - last_iter.l()) /
                             (iter->s() - last_iter.s());
}
void SLPolygon::PrintToLog(std::string prefix) const {
  PrintCurves print_curve;
  std::string key = id_ + prefix + "_sl_boundary";
  for (int i = 0; i < sl_boundary_.boundary_point_size(); i++) {
    print_curve.AddPoint(key, sl_boundary_.boundary_point(i).s(),
                         sl_boundary_.boundary_point(i).l());
  }
  print_curve.PrintToLog();
}

void SLPolygon::PrintToLogBlock() const {
  PrintCurves print_curve;
  std::string key = id_ + "_BlockSLPolygons";
  for (int i = 0; i < sl_boundary_.boundary_point_size(); i++) {
    print_curve.AddPoint(key, sl_boundary_.boundary_point(i).s(),
                         sl_boundary_.boundary_point(i).l());
  }
  print_curve.PrintToLog();
}

SLPolygon::SLPolygon(SLBoundary sl_boundary, std::string id,
                     PerceptionObstacle::Type type, bool print_log)
    : sl_boundary_(sl_boundary), id_(id), obstacle_type_(type) {
  int min_s_index = -1;
  int max_s_index = -1;
  int min_l_index = -1;
  int max_l_index = -1;
  double min_s = std::numeric_limits<double>::max();
  double min_l = std::numeric_limits<double>::max();
  double max_s = std::numeric_limits<double>::lowest();
  double max_l = std::numeric_limits<double>::lowest();
  for (int i = 0; i < sl_boundary.boundary_point_size(); i++) {
    const auto& sl_point = sl_boundary.boundary_point(i);
    if (sl_point.s() < min_s) {
      min_s = sl_point.s();
      min_s_index = i;
    }
    if (sl_point.s() > max_s) {
      max_s = sl_point.s();
      max_s_index = i;
    }
    if (sl_point.l() < min_l) {
      min_l = sl_point.l();
      min_l_index = i;
    }
    if (sl_point.l() > max_l) {
      max_l = sl_point.l();
      max_l_index = i;
    }
  }
  min_s_point_ = sl_boundary.boundary_point(min_s_index);
  min_l_point_ = sl_boundary.boundary_point(min_l_index);
  max_s_point_ = sl_boundary.boundary_point(max_s_index);
  max_l_point_ = sl_boundary.boundary_point(max_l_index);
  // AINFO << "min_s_point_" << min_s_point_.s() << "," << min_s_point_.l();
  // AINFO << "min_l_point_" << min_l_point_.s() << "," << min_l_point_.l();
  // AINFO << "max_s_point_" << max_s_point_.s() << "," << max_s_point_.l();
  // AINFO << "max_l_point_" << max_l_point_.s() << "," << max_l_point_.l();
  int t = min_s_index;
  SLPoint sl_point;
  while (t != max_s_index) {
    sl_point.set_s(sl_boundary.boundary_point(t).s());
    sl_point.set_l(sl_boundary.boundary_point(t).l());
    right_boundary_.push_back(sl_point);
    t = (t + 1) % sl_boundary.boundary_point_size();
  }
  sl_point.set_s(sl_boundary.boundary_point(t).s());
  sl_point.set_l(sl_boundary.boundary_point(t).l());
  right_boundary_.push_back(sl_point);
  if (right_boundary_.front().s() > right_boundary_.back().s()) {
    std::reverse(right_boundary_.begin(), right_boundary_.end());
  }

  t = max_s_index;
  while (t != min_s_index) {
    sl_point.set_s(sl_boundary.boundary_point(t).s());
    sl_point.set_l(sl_boundary.boundary_point(t).l());
    left_boundary_.push_back(sl_point);
    t = (t + 1) % sl_boundary.boundary_point_size();
  }
  sl_point.set_s(sl_boundary.boundary_point(t).s());
  sl_point.set_l(sl_boundary.boundary_point(t).l());
  left_boundary_.push_back(sl_point);
  if (left_boundary_.front().s() > left_boundary_.back().s()) {
    std::reverse(left_boundary_.begin(), left_boundary_.end());
  }
  double mid_s = (sl_boundary.boundary_point(min_s_index).s() +
                  sl_boundary.boundary_point(max_s_index).s()) /
                 2.0;

  if (GetInterpolatedLFromBoundary(left_boundary_, mid_s) <
      GetInterpolatedLFromBoundary(right_boundary_, mid_s)) {
    std::swap(left_boundary_, right_boundary_);
  }

  if (print_log) {
    PrintCurves print_curve;
    for (auto pt : right_boundary_) {
      print_curve.AddPoint("right_boundary", pt.s(), pt.l());
    }
    for (auto pt : left_boundary_) {
      print_curve.AddPoint("left_boundary", pt.s(), pt.l());
    }
    print_curve.PrintToLog();
  }
}

double SLPolygon::GetInterpolatedSFromBoundary(
    const std::vector<SLPoint>& boundary, double s) {
  if (s <= boundary.front().s()) {
    return boundary.front().l();
  }
  if (s >= boundary.back().s()) {
    return boundary.back().l();
  }
  auto iter = std::lower_bound(
      boundary.begin(), boundary.end(), s,
      [](const SLPoint& sl_point, const double s) { return sl_point.s() < s; });
  auto last_iter = std::prev(iter);
  double ret = last_iter->l() + (s - last_iter->s()) *
                                    (iter->l() - last_iter->l()) /
                                    (iter->s() - last_iter->s());
  return ret;
}

void SLPolygon::UpdatePassableInfo(double left_bound, double right_bound,
                                   double left_buffer, double right_buffer,
                                   double check_s) {
  if (!is_passable_[0] && !is_passable_[1]) {
    return;
  }
  double l_lower = GetRightBoundaryByS(check_s);
  double l_upper = GetLeftBoundaryByS(check_s);
  is_passable_[0] = left_bound > l_upper + left_buffer;
  is_passable_[1] = right_bound < l_lower - right_buffer;
}

double SLPolygon::MinRadiusStopDistance(double adc_min_l, double adc_max_l,
                                        double ego_half_width) {
  if (min_radius_stop_distance_ > 0) {
    return min_radius_stop_distance_;
  }
  static constexpr double stop_distance_buffer = 0.4;
  double min_turn_radius = VehicleConfigHelper::MinSafeTurnRadius();
  AINFO << "min_turn_radius: " << min_turn_radius;

  const auto& adc_param =
      VehicleConfigHelper::Instance()->GetConfig().vehicle_param();
  double lateral_diff = 0.0;
  double expand_adc_half_width =
      ego_half_width + FLAGS_nonstatic_obstacle_nudge_l_buffer;
  min_turn_radius += expand_adc_half_width;
  AINFO << "expand min_turn_radius: " << min_turn_radius;
  if (nudge_type_ == NudgeType::LEFT_NUDGE) {
    lateral_diff =
        max_l_point_.l() - adc_min_l + FLAGS_nonstatic_obstacle_nudge_l_buffer;
  } else if (nudge_type_ == NudgeType::RIGHT_NUDGE) {
    lateral_diff =
        adc_max_l + FLAGS_nonstatic_obstacle_nudge_l_buffer - min_l_point_.l();
  }
  lateral_diff = std::max(0.0, lateral_diff);
  const double kEpison = 1e-5;
  lateral_diff = std::min(lateral_diff, min_turn_radius - kEpison);
  AINFO << "obs: " << id_ << ", lateral_diff: " << lateral_diff;
  double min_radius_stop_distance_ =
      std::sqrt(std::fabs(min_turn_radius * min_turn_radius -
                          (min_turn_radius - lateral_diff) *
                              (min_turn_radius - lateral_diff))) +
      stop_distance_buffer;
  double turn_heading =
      std::atan2(min_radius_stop_distance_, min_turn_radius - lateral_diff);

  min_radius_stop_distance_ +=
      adc_param.front_edge_to_center() * std::cos(turn_heading);
  min_radius_stop_distance_ =
      std::min(min_radius_stop_distance_, FLAGS_max_stop_distance_obstacle);
  min_radius_stop_distance_ =
      std::max(min_radius_stop_distance_, FLAGS_min_stop_distance_obstacle +
                                              adc_param.front_edge_to_center());
  AINFO << "obs: " << id_
        << ", min_radius_stop_distance: " << min_radius_stop_distance_;
  return min_radius_stop_distance_;
}

}  // namespace planning
}  // namespace apollo
