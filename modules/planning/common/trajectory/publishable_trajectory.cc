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
 * @file publishable_trajectory.cpp
 **/

#include "modules/planning/common/trajectory/publishable_trajectory.h"

namespace apollo {
namespace planning {

using apollo::common::TrajectoryPoint;

TrajectoryPoint PublishableTrajectory::evaluate_absolute_time(
    const double abs_time) const {
  return evaluate(abs_time - _header_time);
}

TrajectoryPoint
PublishableTrajectory::evaluate_linear_approximation_absolute_time(
    const double abs_time) const {
  return evaluate_linear_approximation(abs_time - _header_time);
}

std::uint32_t PublishableTrajectory::query_nearest_point_absolute_time(
    const double abs_time) const {
  return query_nearest_point(abs_time - _header_time);
}

double PublishableTrajectory::header_time() const { return _header_time; }

void PublishableTrajectory::set_header_time(const double header_time) {
  _header_time = header_time;
}

ADCTrajectory PublishableTrajectory::to_trajectory_protobuf() const {
  ADCTrajectory trajectory_pb;
  trajectory_pb.mutable_header()->set_timestamp_sec(_header_time);
  for (const auto& tp : _trajectory_points) {
    auto* trajectory_point = trajectory_pb.add_trajectory_point();
    trajectory_point->CopyFrom(tp);
  }
  return trajectory_pb;
}

}  // namespace planning
}  // namespace apollo
