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
#pragma once

#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace apollo {
namespace planning {

#define LEFT_INDEX 0
#define RIGHT_INDEX 1

enum BoundType { ROAD = 0, LANE = 1, OBSTACLE = 2, ADC = 3 };
struct BoundEdge {
  BoundType type;
  double l = 0;
  std::string id;
};

struct InterPolatedPoint {
  InterPolatedPoint(double left_weight, double right_weight, double lower_bound,
                    double upper_bound, size_t left_index, size_t right_index,
                    double rear_axle_s, std::string id = "")
      : left_weight(left_weight),
        right_weight(right_weight),
        lower_bound(lower_bound),
        upper_bound(upper_bound),
        left_index(left_index),
        right_index(right_index),
        rear_axle_s(rear_axle_s),
        id(id) {}

  double left_weight;
  double right_weight;
  double lower_bound;
  double upper_bound;
  size_t left_index;
  size_t right_index;
  double rear_axle_s;
  std::string id;
};

class ObsCornerConstraints : public std::vector<InterPolatedPoint> {
 public:
  std::string blocked_id;
  size_t block_left_index;
  size_t block_right_index;
};

class ADCVertexConstraints : public std::vector<InterPolatedPoint> {
 public:
  double front_edge_to_center;
};

struct PathBoundPoint {
  PathBoundPoint() {}
  PathBoundPoint(double s_init, double l_min, double l_max) {
    s = s_init;
    l_lower.l = l_min;
    l_upper.l = l_max;
  }
  bool operator<(const PathBoundPoint& other) const { return s < other.s; }
  double width() const { return  l_upper.l - l_lower.l; }
  bool is_passable() const { return l_upper.l - l_lower.l > 0; }
  double s = 0;
  double towing_l = 0;
  BoundEdge l_lower;
  BoundEdge l_upper;
  bool is_nudge_bound[2] = {false, false};
};

using PathBound = std::vector<PathBoundPoint>;

class PathBoundary : public std::vector<PathBoundPoint> {
 public:
  PathBoundary() {}
  PathBoundary(const double start_s, const double delta_s,
               std::vector<std::pair<double, double>> path_boundary);
  /**
   * @brief construct class by path bound
   * @param delta_s The distance between two point in s-axis
   * @param path_bound tuple first is s, second is l_min, third is l_max
   *    **/
  PathBoundary(const double delta_s, const PathBound& path_bound);
  virtual ~PathBoundary() = default;
  void set_delta_s(double s);
  double start_s() const;

  double delta_s() const;

  void set_boundary(const std::vector<std::pair<double, double>>& boundary);
  std::vector<std::pair<double, double>> boundary() const;
  bool get_interpolated_s_weight(const double s, double* left_weight,
                                 double* right_weight, size_t* left_index,
                                 size_t* right_index) const;
  double get_lower_bound_by_s(const double s);
  double get_upper_bound_by_s(const double s);
  double get_lower_bound_by_interpolated_index(double left_weight,
                                               double right_weight,
                                               size_t left_index,
                                               size_t right_index) const;
  double get_upper_bound_by_interpolated_index(double left_weight,
                                               double right_weight,
                                               size_t left_index,
                                               size_t right_index) const;
  void set_label(const std::string& label);
  const std::string& label() const;

  void set_blocking_obstacle_id(const std::string& obs_id);
  const std::string& blocking_obstacle_id() const;
  void DebugString(std::string name);
  ObsCornerConstraints* mutable_extra_path_bound() {
    return &extra_path_bound_;
  }
  const ObsCornerConstraints& extra_path_bound() const {
    return extra_path_bound_;
  }
  ADCVertexConstraints* mutable_adc_vertex_bound() {
    return &adc_vertex_bound_;
  }
  const ADCVertexConstraints& adc_vertex_bound() const {
    return adc_vertex_bound_;
  }

 private:
  double start_s_ = 0.0;
  double delta_s_ = 0.0;
  ObsCornerConstraints extra_path_bound_;
  ADCVertexConstraints adc_vertex_bound_;
  std::string label_ = "regular";
  std::string blocking_obstacle_id_ = "";
};

}  // namespace planning
}  // namespace apollo
