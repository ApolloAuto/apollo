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

#pragma once

#include <vector>

#include "modules/common_msgs/basic_msgs/pnc_point.pb.h"
#include "modules/common/status/status.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/reference_line_info.h"
#include "modules/planning/common/speed_limit.h"
#include "modules/planning/proto/auto_tuning_raw_feature.pb.h"

namespace apollo {
namespace planning {

class AutotuningRawFeatureGenerator {
 public:
  // @brief class constructor
  AutotuningRawFeatureGenerator(const double time_range,
                                const size_t num_points,
                                const ReferenceLineInfo& reference_line_info,
                                const Frame& frame,
                                const SpeedLimit& speed_limit);

  // @evaluation interface
  /**
   * @brief evaluate trajectory with environment, generate model trajectory
   *        features
   * @param reference line related info
   * @param frame related status
   * @return OK if the evaluation succeeds; error otherwise.
   */
  common::Status EvaluateTrajectory(
      const std::vector<common::TrajectoryPoint>& trajectory,
      autotuning::TrajectoryRawFeature* const trajectory_feature) const;

  /**
   * @brief evaluate trajectory point with environment, generate model
   * trajectory
   *        features
   * @param reference line related info
   * @param frame related status
   * @return OK if the evaluation succeeds; error otherwise.
   */
  common::Status EvaluateTrajectoryPoint(
      const common::TrajectoryPoint& trajectory_point,
      autotuning::TrajectoryPointRawFeature* const trajectory_point_feature)
      const;

  /**
   * EvaluateSpeed Profile shall match the time range as well as resolution
   */
  common::Status EvaluateSpeedProfile(
      const std::vector<common::SpeedPoint>& speed_profile,
      autotuning::TrajectoryRawFeature* const trajectory_feature) const;

 private:
  void GenerateSTBoundaries(const ReferenceLineInfo& reference_line_info);

  /**
   * Convert st boundaries to discretized boundaries
   */
  void ConvertToDiscretizedBoundaries(const STBoundary& boundary,
                                      const double speed);

  common::Status EvaluateSpeedPoint(const common::SpeedPoint& speed_point,
                                    const size_t index,
                                    autotuning::TrajectoryPointRawFeature* const
                                        trajectory_point_feature) const;

 private:
  std::vector<double> eval_time_;
  const ReferenceLineInfo& reference_line_info_;
  const Frame& frame_;
  const SpeedLimit& speed_limit_;
  std::vector<const STBoundary*> boundaries_;

  // covers the boundary info lower s upper s as well as the speed of obs
  std::vector<std::vector<std::array<double, 3>>> obs_boundaries_;
  std::vector<std::vector<std::array<double, 3>>> stop_boundaries_;
  std::vector<std::vector<std::array<double, 3>>> nudge_boundaries_;
  std::vector<std::vector<std::array<double, 3>>> side_pass_boundaries_;
};

}  // namespace planning
}  // namespace apollo
