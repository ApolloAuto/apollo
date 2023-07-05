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

#include "cyber/common/log.h"

namespace apollo {
namespace planning {

PublishableTrajectory::PublishableTrajectory(
    const double header_time,
    const DiscretizedTrajectory& discretized_trajectory)
    : DiscretizedTrajectory(discretized_trajectory),
      header_time_(header_time) {}

PublishableTrajectory::PublishableTrajectory(const ADCTrajectory& trajectory_pb)
    : DiscretizedTrajectory(trajectory_pb),
      header_time_(trajectory_pb.header().timestamp_sec()) {}

double PublishableTrajectory::header_time() const { return header_time_; }

void PublishableTrajectory::PopulateTrajectoryProtobuf(
    ADCTrajectory* trajectory_pb) const {
  CHECK_NOTNULL(trajectory_pb);
  trajectory_pb->mutable_header()->set_timestamp_sec(header_time_);
  trajectory_pb->mutable_trajectory_point()->CopyFrom({begin(), end()});
  if (!empty()) {
    const auto& last_tp = back();
    trajectory_pb->set_total_path_length(last_tp.path_point().s());
    trajectory_pb->set_total_path_time(last_tp.relative_time());
  }
}

}  // namespace planning
}  // namespace apollo
