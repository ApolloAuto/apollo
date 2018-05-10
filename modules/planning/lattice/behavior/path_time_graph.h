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

#ifndef MODULES_PLANNING_LATTICE_BEHAVIOR_PATH_TIME_GRAPH_H_
#define MODULES_PLANNING_LATTICE_BEHAVIOR_PATH_TIME_GRAPH_H_

#include <array>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "modules/common/proto/geometry.pb.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/proto/lattice_structure.pb.h"
#include "modules/planning/reference_line/reference_line.h"

namespace apollo {
namespace planning {

class PathTimeGraph {
 public:
  PathTimeGraph(const std::vector<const Obstacle*>& obstacles,
                const std::vector<common::PathPoint>& discretized_ref_points,
                const double s_start, const double s_end, const double t_start,
                const double t_end, const double path_width);

  const std::vector<PathTimeObstacle>& GetPathTimeObstacles() const;

  bool GetPathTimeObstacle(const std::string& obstacle_id,
                           PathTimeObstacle* path_time_obstacle);

  std::vector<std::pair<double, double>> GetPathBlockingIntervals(
      const double t) const;

  std::vector<std::vector<std::pair<double, double>>> GetPathBlockingIntervals(
      const double t_start, const double t_end, const double t_resolution);

  std::pair<double, double> get_path_range() const;

  std::pair<double, double> get_time_range() const;

  std::vector<PathTimePoint> GetObstacleSurroundingPoints(
      const std::string& obstacle_id, const double s_dist,
      const double t_density) const;

 private:
  void SetupObstacles(
      const std::vector<const Obstacle*>& obstacles,
      const std::vector<common::PathPoint>& discretized_ref_points);

  SLBoundary ComputeObstacleBoundary(
      const common::math::Box2d& box,
      const std::vector<common::PathPoint>& discretized_ref_points) const;

  PathTimePoint SetPathTimePoint(const std::string& obstacle_id, const double s,
                                 const double t) const;

  void SetStaticObstacle(
      const Obstacle* obstacle,
      const std::vector<common::PathPoint>& discretized_ref_points);

 private:
  std::pair<double, double> time_range_;

  std::pair<double, double> path_range_;

  std::unordered_map<std::string, PathTimeObstacle> path_time_obstacle_map_;

  std::vector<PathTimeObstacle> path_time_obstacles_;

  double half_path_width_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_LATTICE_BEHAVIOR_PATH_TIME_GRAPH_H_
