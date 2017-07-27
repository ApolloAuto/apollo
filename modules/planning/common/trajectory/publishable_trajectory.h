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
 * @file publishable_trajectory.h
 **/

#ifndef MODULES_PLANNING_COMMON_TRAJECTORY_PUBLISHABLE_TRAJECTORY_H
#define MODULES_PLANNING_COMMON_TRAJECTORY_PUBLISHABLE_TRAJECTORY_H

#include "modules/planning/common/trajectory/discretized_trajectory.h"
#include "modules/planning/proto/planning.pb.h"

namespace apollo {
namespace planning {

class PublishableTrajectory : public DiscretizedTrajectory {
public:
  virtual ~PublishableTrajectory() = default;

  apollo::common::TrajectoryPoint evaluate_absolute_time(
      const double abs_time) const;

  apollo::common::TrajectoryPoint evaluate_linear_approximation_absolute_time(
      const double abs_time) const;

  std::uint32_t query_nearest_point_absolute_time(const double abs_time) const;

  double header_time() const;

  void set_header_time(const double header_time);

  ADCTrajectory to_trajectory_protobuf() const;
  void populate_trajectory_protobuf(ADCTrajectory* trajectory_pb) const;

private:
  double _header_time;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_COMMON_TRAJECTORY_PUBLISHABLE_TRAJECTORY_H
