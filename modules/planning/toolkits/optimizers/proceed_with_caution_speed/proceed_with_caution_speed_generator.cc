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

#include "modules/planning/toolkits/optimizers/proceed_with_caution_speed/proceed_with_caution_speed_generator.h"

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include "modules/common/proto/pnc_point.pb.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/util/file.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/speed_profile_generator.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::planning_internal::STGraphDebug;

namespace {
const double proceeding_speed = 2.23;    // (5mph proceeding speed)
const double const_deceleration = -0.8;  // (~3sec to fully stop)
const double increment_s = 0.1;
const double increment_t = 0.1;
}  // namespace

ProceedWithCautionSpeedGenerator::ProceedWithCautionSpeedGenerator(
    const TaskConfig& config)
    : SpeedOptimizer(config) {
  CHECK(config_.has_proceed_with_caution_speed_config());
  const auto& proceed_with_caution_speed_config =
      config_.proceed_with_caution_speed_config();
  is_fixed_distance_ = proceed_with_caution_speed_config.type();
  max_distance_or_speed_ =
      is_fixed_distance_ ? proceed_with_caution_speed_config.max_distance()
                         : proceed_with_caution_speed_config.max_speed();
  SetName("ProceedWithCautionSpeedGenerator");
}

Status ProceedWithCautionSpeedGenerator::Process(
    const SLBoundary& adc_sl_boundary, const PathData& path_data,
    const TrajectoryPoint& init_point, const ReferenceLine& reference_line,
    const SpeedData& reference_speed_data, PathDecision* const path_decision,
    SpeedData* const speed_data) {
  if (path_data.discretized_path().NumOfPoints() == 0) {
    std::string msg("Empty path data");
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  speed_data->Clear();
  double tot_len = path_data.discretized_path().Length();

  if (is_fixed_distance_) {
    if (tot_len < max_distance_or_speed_) {
      std::string msg("The length planned by path data is too small.");
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
    *speed_data = SpeedProfileGenerator::GenerateFixedDistanceCreepProfile(
        tot_len, proceeding_speed);
  } else {
    if (proceeding_speed > max_distance_or_speed_) {
      std::string msg("Speed exceeds the max allowed value.");
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
    *speed_data = SpeedProfileGenerator::GenerateFixedSpeedCreepProfile(
        tot_len, proceeding_speed);
  }
  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
