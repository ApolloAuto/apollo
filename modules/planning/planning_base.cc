/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/planning_base.h"

#include <algorithm>
#include <list>
#include <vector>

#include "modules/common/time/time.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::TrajectoryPoint;
using apollo::common::time::Clock;
using apollo::hdmap::HDMapUtil;

PlanningBase::~PlanningBase() {}

void PlanningBase::FillPlanningPb(const double timestamp,
                                  ADCTrajectory* const trajectory_pb) {
  trajectory_pb->mutable_header()->set_timestamp_sec(timestamp);
  if (local_view_.prediction_obstacles != nullptr &&
      !local_view_.prediction_obstacles->has_header()) {
    trajectory_pb->mutable_header()->set_lidar_timestamp(
        local_view_.prediction_obstacles->header().lidar_timestamp());
    trajectory_pb->mutable_header()->set_camera_timestamp(
        local_view_.prediction_obstacles->header().camera_timestamp());
    trajectory_pb->mutable_header()->set_radar_timestamp(
        local_view_.prediction_obstacles->header().radar_timestamp());
  }

  // TODO(all): integrate reverse gear
  trajectory_pb->set_gear(canbus::Chassis::GEAR_DRIVE);

  if (FLAGS_use_planning_fallback &&
      trajectory_pb->trajectory_point_size() == 0) {
    SetFallbackTrajectory(trajectory_pb);
  }

  // NOTICE:
  // Since we are using the time at each cycle beginning as timestamp, the
  // relative time of each trajectory point should be modified so that we can
  // use the current timestamp in header.

  // auto* trajectory_points = trajectory_pb.mutable_trajectory_point();
  if (!FLAGS_planning_test_mode) {
    const double dt = timestamp - Clock::NowInSeconds();
    for (auto& p : *trajectory_pb->mutable_trajectory_point()) {
      p.set_relative_time(p.relative_time() + dt);
    }
  }
}

void PlanningBase::SetFallbackTrajectory(ADCTrajectory* const trajectory_pb) {
  CHECK_NOTNULL(trajectory_pb);
  // use planning trajectory from last cycle
  if (last_planning_ != nullptr) {
    const double current_time_stamp = trajectory_pb->header().timestamp_sec();
    const double pre_time_stamp = last_planning_->header().timestamp_sec();

    for (int i = 0; i < last_planning_->trajectory_point_size(); ++i) {
      const double t = last_planning_->trajectory_point(i).relative_time() +
                       pre_time_stamp - current_time_stamp;
      auto* p = trajectory_pb->add_trajectory_point();
      p->CopyFrom(last_planning_->trajectory_point(i));
      p->set_relative_time(t);
    }
  }
}

}  // namespace planning
}  // namespace apollo
