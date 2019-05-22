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

#include <utility>
#include <vector>

#include "Eigen/Eigen"

#include "modules/common/math/vec2d.h"
#include "modules/planning/common/path/discretized_path.h"
#include "modules/planning/common/speed/speed_data.h"
#include "modules/planning/common/trajectory/discretized_trajectory.h"

namespace apollo {
namespace planning {
class IterativeAnchoringSmoother {
 public:
  IterativeAnchoringSmoother() = default;

  ~IterativeAnchoringSmoother() = default;

  bool Smooth(const Eigen::MatrixXd& xWS, const double init_a,
              const double init_v,
              const std::vector<std::vector<common::math::Vec2d>>&
                  obstacles_vertices_vec,
              DiscretizedTrajectory* discretized_trajectory);

 private:
  bool SmoothPath(const DiscretizedPath& raw_path_points,
                  const std::vector<double>& bounds,
                  const std::vector<std::vector<common::math::Vec2d>>&
                      obstacles_vertices_vec,
                  DiscretizedPath* smoothed_path_points);

  bool CheckCollision(const DiscretizedPath& path_points,
                      const std::vector<std::vector<common::math::Vec2d>>&
                          obstacles_vertices_vec,
                      std::vector<size_t>* colliding_point_index);

  void AdjustPathBounds(const std::vector<size_t>& colliding_point_index,
                        std::vector<double>* bounds);

  bool SetPathProfile(const std::vector<std::pair<double, double>>& point2d,
                      DiscretizedPath* raw_path_points);

  bool CheckGear(const Eigen::MatrixXd& xWS);

  bool SmoothSpeed(const double init_a, const double init_v,
                   const double path_length, SpeedData* smoothed_speeds);

  bool CombinePathAndSpeed(const DiscretizedPath& raw_path_points,
                           const SpeedData& smoothed_speeds,
                           DiscretizedTrajectory* discretized_trajectory);

 private:
  // gear DRIVE as true and gear REVERSE as false
  bool gear_ = false;
};
}  // namespace planning
}  // namespace apollo
