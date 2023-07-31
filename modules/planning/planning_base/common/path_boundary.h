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
enum BoundType { ROAD = 0, LANE = 1, OBSTACLE = 2, ADC = 3 };
struct BoundEdge {
  BoundType type;
  double l = 0;
  std::string id;
};

struct PathBoundPoint {
  PathBoundPoint(double s_init, double l_min, double l_max) {
    s = s_init;
    l_lower.l = l_min;
    l_upper.l = l_max;
  }
  double s = 0;
  BoundEdge l_lower;
  BoundEdge l_upper;
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

  void set_label(const std::string& label);
  const std::string& label() const;

  void set_blocking_obstacle_id(const std::string& obs_id);
  const std::string& blocking_obstacle_id() const;
  void DebugString(std::string name);

 private:
  double start_s_ = 0.0;
  double delta_s_ = 0.0;
  std::string label_ = "regular";
  std::string blocking_obstacle_id_ = "";
};

}  // namespace planning
}  // namespace apollo
