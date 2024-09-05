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

/**
 * @file
 **/

#include "modules/planning/planning_base/common/path_boundary.h"

#include <tuple>

#include "cyber/common/log.h"
#include "modules/planning/planning_base/common/util/print_debug_info.h"
namespace apollo {
namespace planning {

double PathBoundary::start_s() const {
  if (size() == 0) {
    return 0;
  } else {
    return begin()->s;
  }
}

double PathBoundary::delta_s() const { return delta_s_; }
void apollo::planning::PathBoundary::set_delta_s(double s) { delta_s_ = s; }

std::vector<std::pair<double, double>> PathBoundary::boundary() const {
  std::vector<std::pair<double, double>> boundary;
  for (auto iter = begin(); iter != end(); iter++) {
    boundary.emplace_back(iter->l_lower.l, iter->l_upper.l);
  }
  return boundary;
}

void PathBoundary::set_label(const std::string& label) { label_ = label; }

const std::string& PathBoundary::label() const { return label_; }

void PathBoundary::set_blocking_obstacle_id(const std::string& obs_id) {
  blocking_obstacle_id_ = obs_id;
}
void PathBoundary::DebugString(std::string name) {
  PrintCurves print_curves;
  std::string lower_name = name + "_lower";
  std::string upper_name = name + "_upper";
  for (auto iter = begin(); iter != end(); iter++) {
    print_curves.AddPoint(lower_name, iter->s, iter->l_lower.l);
    print_curves.AddPoint(upper_name, iter->s, iter->l_upper.l);
  }
  print_curves.PrintToLog();
}

const std::string& PathBoundary::blocking_obstacle_id() const {
  return blocking_obstacle_id_;
}

bool PathBoundary::get_interpolated_s_weight(const double s,
                                             double* left_weight,
                                             double* right_weight,
                                             size_t* left_index,
                                             size_t* right_index) const {
  if (s < front().s || s > back().s) {
    return false;
  }
  PathBoundPoint cmp(s, 0.0, 0.0);
  const auto iter = std::lower_bound(begin(), end(), cmp);
  auto last_iter = std::prev(iter);
  *left_index = std::distance(begin(), last_iter);
  *right_index = *left_index + 1;
  if (*left_index > size() - 1 || *right_index > size() - 1) {
    return false;
  }
  *left_weight =
      (at(*right_index).s - s) / (at(*right_index).s - at(*left_index).s);
  *right_weight = 1.0 - *left_weight;
  return true;
}

double PathBoundary::get_lower_bound_by_s(const double s) {
  if (s <= front().s) {
    return front().l_lower.l;
  }
  if (s >= back().s) {
    return back().l_lower.l;
  }
  double l_weight = 0.0;
  double r_weight = 0.0;
  size_t l_index = 0;
  size_t r_index = 0;
  get_interpolated_s_weight(s, &l_weight, &r_weight, &l_index, &r_index);

  return at(l_index).l_lower.l * l_weight + at(r_index).l_lower.l * r_weight;
}

double PathBoundary::get_upper_bound_by_s(const double s) {
  if (s <= front().s) {
    return front().l_upper.l;
  }
  if (s >= back().s) {
    return back().l_upper.l;
  }
  double l_weight = 0.0;
  double r_weight = 0.0;
  size_t l_index = 0;
  size_t r_index = 0;
  get_interpolated_s_weight(s, &l_weight, &r_weight, &l_index, &r_index);

  return at(l_index).l_upper.l * l_weight + at(r_index).l_upper.l * r_weight;
}

double PathBoundary::get_lower_bound_by_interpolated_index(
    double left_weight, double right_weight, size_t left_index,
    size_t right_index) const {
  if (left_index < 0) {
    return front().l_lower.l;
  } else if (right_index >= size()) {
    return back().l_lower.l;
  }
  return at(left_index).l_lower.l * left_weight +
         at(right_index).l_lower.l * right_weight;
}

double PathBoundary::get_upper_bound_by_interpolated_index(
    double left_weight, double right_weight, size_t left_index,
    size_t right_index) const {
  if (left_index < 0) {
    return front().l_upper.l;
  } else if (right_index >= size()) {
    return back().l_upper.l;
  }
  return at(left_index).l_upper.l * left_weight +
         at(right_index).l_upper.l * right_weight;
}

}  // namespace planning
}  // namespace apollo
