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

#include "modules/common/status/status.h"
#include "modules/planning/proto/st_boundary_config.pb.h"
#include "modules/planning/tasks/optimizers/speed_optimizer.h"

namespace apollo {
namespace planning {

/*
 * @class ProceedWithCautionSpeedGenerator
 * @brief
 * --- (1) ProceedWidthCautionSpeedGenerator generate speed profiles
 * (represented by a list of speed points) when the ego vehicle need to proceed
 * with a low speed.
 * --- (2) This class does NOT check any collision, so that the caller is
 * responsible to guarantee safety before calling this class.
 * --- (3) This class only guarantee the max speed of the generated profile
 * doesn't exceed a certain limit.
 */
class ProceedWithCautionSpeedGenerator : public SpeedOptimizer {
 public:
  explicit ProceedWithCautionSpeedGenerator(const TaskConfig& config);

 private:
  common::Status Process(const SLBoundary& adc_sl_boundary,
                         const PathData& path_data,
                         const apollo::common::TrajectoryPoint& init_point,
                         const ReferenceLine& reference_line,
                         const SpeedData& reference_speed_data,
                         PathDecision* const path_decision,
                         SpeedData* const speed_data) override;
 private:
  // TODO(all): might move to conf later
  const double proceeding_speed_ = 2.23;    // (5mph proceeding speed)
  const double const_deceleration_ = -0.8;  // (~3sec to fully stop)
  const double increment_s_ = 0.1;
  const double increment_t_ = 0.1;
};

}  // namespace planning
}  // namespace apollo
