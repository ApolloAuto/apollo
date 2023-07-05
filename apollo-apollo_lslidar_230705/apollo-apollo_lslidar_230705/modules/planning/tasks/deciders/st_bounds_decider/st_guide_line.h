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

/**
 *   @file
 **/

#pragma once

#include <vector>

#include "modules/common_msgs/config_msgs/vehicle_config.pb.h"
#include "modules/common_msgs/basic_msgs/pnc_point.pb.h"
#include "modules/common/status/status.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/common/path/path_data.h"
#include "modules/planning/common/path_decision.h"
#include "modules/planning/common/speed/speed_data.h"
#include "modules/planning/common/speed/st_boundary.h"
#include "modules/planning/common/speed_limit.h"
#include "modules/planning/common/trajectory/discretized_trajectory.h"
#include "modules/planning/reference_line/reference_line.h"

namespace apollo {
namespace planning {

// TODO(jiacheng): currently implemented a constant velocity model for
// guide-line. Upgrade it to a constant acceleration model.
class STGuideLine {
 public:
  STGuideLine() {}

  void Init(double desired_v);

  void Init(double desired_v,
            const std::vector<common::TrajectoryPoint> &speed_reference);

  virtual ~STGuideLine() = default;

  double GetGuideSFromT(double t);

  void UpdateBlockingInfo(const double t, const double s_block,
                          const bool is_lower_block);

 private:
  // Variables for simple guide-line calculation.
  double t0_;
  double s0_;
  double v0_;
  // St guideline from upstream modules
  SpeedData guideline_speed_data_;
};

}  // namespace planning
}  // namespace apollo
