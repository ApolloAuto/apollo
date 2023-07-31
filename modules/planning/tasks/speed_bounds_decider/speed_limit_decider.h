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
 *   @file
 **/

#pragma once

#include <string>
#include <vector>

#include "modules/common_msgs/config_msgs/vehicle_config.pb.h"
#include "modules/planning/tasks/speed_bounds_decider/proto/speed_bounds_decider.pb.h"
#include "modules/common/status/status.h"
#include "modules/planning/planning_base/common/obstacle.h"
#include "modules/planning/planning_base/common/path/path_data.h"
#include "modules/planning/planning_base/common/speed_limit.h"
#include "modules/planning/planning_base/reference_line/reference_line.h"

namespace apollo {
namespace planning {

class SpeedLimitDecider {
 public:
  SpeedLimitDecider(const SpeedBoundsDeciderConfig& config,
                    const ReferenceLine& reference_line,
                    const PathData& path_data);

  virtual ~SpeedLimitDecider() = default;

  virtual common::Status GetSpeedLimits(
      const IndexedList<std::string, Obstacle>& obstacles,
      SpeedLimit* const speed_limit_data) const;

 private:
  FRIEND_TEST(SpeedLimitDeciderTest, get_centric_acc_limit);
  double GetCentricAccLimit(const double kappa) const;

  void GetAvgKappa(const std::vector<common::PathPoint>& path_points,
                   std::vector<double>* kappa) const;

 private:
  const SpeedBoundsDeciderConfig& speed_bounds_config_;
  const ReferenceLine& reference_line_;
  const PathData& path_data_;
  const apollo::common::VehicleParam& vehicle_param_;
};

}  // namespace planning
}  // namespace apollo
