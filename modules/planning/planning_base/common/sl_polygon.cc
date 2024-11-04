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

namespace apollo {
namespace planning {
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

SLPolygon::SLPolygon(SLBoundary sl_boundary, std::string id, bool print_log)
    : sl_boundary_(sl_boundary), id_(id) {
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
  return last_iter->l() + (s - last_iter->s()) * (iter->l() - last_iter->l()) /
                              (iter->s() - last_iter->s());
}

}  // namespace planning
}  // namespace apollo
