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

/*
 * @file
 */

#pragma once

#include <memory>
#include <queue>
#include <unordered_map>
#include <utility>
#include <vector>
#include <algorithm>
#include <limits>

#include "cyber/common/log.h"
#include "modules/common/math/line_segment2d.h"
#include "modules/planning/proto/planner_open_space_config.pb.h"

namespace apollo {
namespace planning {

class Node2d {
 public:
  Node2d(const double& x, const double& y, const double& xy_resolution,
         const std::vector<double>& XYbounds) {
    // XYbounds with xmin, xmax, ymin, ymax
    grid_x_ = std::round((x - XYbounds[0]) / xy_resolution);
    grid_y_ = std::round((y - XYbounds[2]) / xy_resolution);
    index_ = grid_x_ * (XYbounds[3] - XYbounds[2]) + grid_y_;
  }
  Node2d(const double& grid_x, const double& grid_y,
         const std::vector<double>& XYbounds) {
    grid_x_ = grid_x;
    grid_y_ = grid_y;
    index_ = grid_x_ * (XYbounds[3] - XYbounds[2]) + grid_y_;
  }
  void SetPathCost(const double& path_cost) {
    path_cost_ = path_cost;
    cost_ = path_cost_ + heuristic_;
  }
  void SetHeuristic(const double& heuristic) {
    heuristic_ = heuristic;
    cost_ = path_cost_ + heuristic_;
  }
  void SetCost(const double& cost) { cost_ = cost; }
  void SetPreNode(std::shared_ptr<Node2d> pre_node) { pre_node_ = pre_node; }
  double GetGridX() const { return grid_x_; }
  double GetGridY() const { return grid_y_; }
  double GetPathCost() const { return path_cost_; }
  double GetHeuCost() const { return heuristic_; }
  double GetCost() { return cost_; }
  double GetIndex() const { return index_; }
  std::shared_ptr<Node2d> GetPreNode() { return pre_node_; }
  static double CalcIndex(const double& x, const double& y,
                          const double& xy_resolution,
                          const std::vector<double>& XYbounds) {
    // XYbounds with xmin, xmax, ymin, ymax
    double grid_x = std::round((x - XYbounds[0]) / xy_resolution);
    double grid_y = std::round((y - XYbounds[2]) / xy_resolution);
    return grid_x * (XYbounds[3] - XYbounds[2]) + grid_y;
  }
  bool operator==(const Node2d& right) const {
    return right.GetIndex() == index_;
  }

 private:
  double grid_x_ = 0.0;
  double grid_y_ = 0.0;
  double path_cost_ = 0.0;
  double heuristic_ = 0.0;
  double cost_ = 0.0;
  double index_ = 0.0;
  std::shared_ptr<Node2d> pre_node_ = nullptr;
};

struct GridAStartResult {
  std::vector<double> x;
  std::vector<double> y;
  double path_cost = 0.0;
};

class GridSearch {
 public:
  explicit GridSearch(const PlannerOpenSpaceConfig& open_space_conf);
  virtual ~GridSearch() = default;
  bool GenerateAStarPath(
      const double& sx, const double& sy, const double& ex, const double& ey,
      const std::vector<double>& XYbounds,
      const std::vector<std::vector<common::math::LineSegment2d>>&
          obstacles_linesegments_vec,
      GridAStartResult* result);
  bool GenerateDpMap(
      const double& ex, const double& ey, const std::vector<double>& XYbounds,
      const std::vector<std::vector<common::math::LineSegment2d>>&
          obstacles_linesegments_vec);
  double CheckDpMap(const double& sx, const double& sy);

 private:
  double EuclidDistance(const double& x1, const double& y1, const double& x2,
                        const double& y2);
  std::vector<std::shared_ptr<Node2d>> GenerateNextNodes(
      std::shared_ptr<Node2d> node);
  bool CheckConstraints(std::shared_ptr<Node2d> node);
  void LoadGridAStarResult(GridAStartResult* result);

 private:
  double xy_grid_resolution_ = 0.0;
  double node_radius_ = 0.0;
  std::vector<double> XYbounds_;
  double max_grid_x_ = 0.0;
  double max_grid_y_ = 0.0;
  std::shared_ptr<Node2d> start_node_;
  std::shared_ptr<Node2d> end_node_;
  std::shared_ptr<Node2d> final_node_;
  std::vector<std::vector<common::math::LineSegment2d>>
      obstacles_linesegments_vec_;

  struct cmp {
    bool operator()(const std::pair<double, double>& left,
                    const std::pair<double, double>& right) const {
      return left.second >= right.second;
    }
  };
  std::unordered_map<double, std::shared_ptr<Node2d>> dp_map_;
};
}  // namespace planning
}  // namespace apollo
