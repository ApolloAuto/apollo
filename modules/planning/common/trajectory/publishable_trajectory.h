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

#ifndef MODULES_PLANNING_COMMON_TRAJECTORY_PUBLISHABLE_TRAJECTORY_H_
#define MODULES_PLANNING_COMMON_TRAJECTORY_PUBLISHABLE_TRAJECTORY_H_

#include "modules/planning/common/trajectory/discretized_trajectory.h"
#include "modules/planning/proto/planning.pb.h"
#include "glog/logging.h"

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

  template <typename Iter>
  void prepend_trajectory_points(Iter begin, Iter end) {
    if (!_trajectory_points.empty() && begin != end) {
      CHECK((end - 1)->relative_time() < _trajectory_points.front().relative_time());
    }
    _trajectory_points.insert(_trajectory_points.begin(), begin, end);
  }

  void populate_trajectory_protobuf(ADCTrajectory* trajectory_pb) const;

 private:
  double _header_time;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_COMMON_TRAJECTORY_PUBLISHABLE_TRAJECTORY_H
