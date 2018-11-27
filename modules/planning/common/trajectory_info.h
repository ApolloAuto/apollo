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

/**
 * @file
 **/

#pragma once

#include "modules/planning/common/path/path_data.h"
#include "modules/planning/common/speed/speed_data.h"
#include "modules/planning/common/trajectory/discretized_trajectory.h"

namespace apollo {
namespace planning {

class TrajectoryInfo {
 public:
  TrajectoryInfo() = default;
  ~TrajectoryInfo() = default;

  const PathData& path_data() const { return path_data_; }
  PathData* mutable_path_data() { return &path_data_; }

  const SpeedData& speed_data() const { return speed_data_; }
  SpeedData* mutable_speed_data() { return &speed_data_; }

  const DiscretizedTrajectory& discretized_trajectory() const {
    return discretized_trajectory_;
  }
  DiscretizedTrajectory* mutable_discretized_trajectory() {
    return &discretized_trajectory_;
  }

 private:
  PathData path_data_;
  SpeedData speed_data_;
  DiscretizedTrajectory discretized_trajectory_;
};

}  // namespace planning
}  // namespace apollo
