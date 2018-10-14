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

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::planning_internal::STGraphDebug;

const double proceeding_speed = 2.23;    // (5mph proceeding speed)
const double const_deceleration = -0.8;  // (~3sec to fully stop)
const double increment_s = 0.1;
const double increment_t = 0.1;

ProceedWithCautionSpeedGenerator::ProceedWithCautionSpeedGenerator(
    const TaskConfig& config)
    : SpeedOptimizer(config) {
  CHECK(config.has_proceed_with_caution_speed_config());
  proceed_with_caution_speed_config_ =
      config.proceed_with_caution_speed_config();
  isFixedDistanceNotFixedSpeed = proceed_with_caution_speed_config_.type();
  maxDistanceOrSpeed = isFixedDistanceNotFixedSpeed
                           ? proceed_with_caution_speed_config_.max_distance()
                           : proceed_with_caution_speed_config_.max_speed();
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

  if (isFixedDistanceNotFixedSpeed) {
    if (tot_len < maxDistanceOrSpeed) {
      std::string msg("The length planned by path data is too small.");
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
    tot_len = maxDistanceOrSpeed;
    double Distance2StartDeceleration =
        proceeding_speed * proceeding_speed / const_deceleration / 2;
    bool ConstDecelerationMode = tot_len < Distance2StartDeceleration;
    double a = const_deceleration;
    double t = 0.0;
    double s = 0.0;
    double v = proceeding_speed;

    while (s < tot_len && v > 0) {
      if (ConstDecelerationMode) {
        speed_data->AppendSpeedPoint(s, t, v, a, 0.0);
        t += increment_t;
        double v_new = std::max(0.0, v + a * t);
        s += increment_t * (v + v_new) / 2;
        v = v_new;
      } else {
        speed_data->AppendSpeedPoint(s, t, v, 0.0, 0.0);
        t += increment_t;
        s += increment_t * v;
        if (tot_len - s < Distance2StartDeceleration)
          ConstDecelerationMode = true;
      }
    }
  } else {
    if (proceeding_speed > maxDistanceOrSpeed) {
      std::string msg("Speed exceeds the max allowed value.");
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
    double v = proceeding_speed;
    for (double s = 0.0; s < tot_len; s += increment_s) {
      speed_data->AppendSpeedPoint(s, s / v, v, 0.0, 0.0);
    }
  }

  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
