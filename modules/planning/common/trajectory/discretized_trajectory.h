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
 * @file discretized_trajectory.h
 **/

#ifndef MODULES_PLANNING_COMMON_TRAJECTORY_DISCRETIZED_TRAJECTORY_H
#define MODULES_PLANNING_COMMON_TRAJECTORY_DISCRETIZED_TRAJECTORY_H

#include "Eigen/Core"
#include "modules/planning/common/trajectory/trajectory.h"

namespace apollo {
namespace planning {

class DiscretizedTrajectory : public Trajectory {
public:
  DiscretizedTrajectory() = default;

  DiscretizedTrajectory(
      std::vector<apollo::common::TrajectoryPoint> trajectory_points);

  virtual ~DiscretizedTrajectory() = default;

  virtual double time_length() const override;

  virtual apollo::common::TrajectoryPoint evaluate(
      const double relative_time) const override;

  virtual apollo::common::TrajectoryPoint start_point() const override;

  virtual apollo::common::TrajectoryPoint end_point() const override;

  virtual apollo::common::TrajectoryPoint evaluate_linear_approximation(
      const double relative_time) const;

  virtual std::size_t query_nearest_point(const double relative_time) const;

  virtual std::size_t query_nearest_point(
      const Eigen::Vector2d& position) const;

  virtual void add_trajectory_point(
      const apollo::common::TrajectoryPoint& trajectory_point);

  const apollo::common::TrajectoryPoint& trajectory_point_at(
      const std::size_t index) const;

  std::size_t num_of_points() const;

protected:
  std::vector<apollo::common::TrajectoryPoint> _trajectory_points;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_COMMON_TRAJECTORY_DISCRETIZED_TRAJECTORY_H
