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

#ifndef MODULES_PLANNING_TUNING_AUTOTUNING_RAW_FEATURE_GENERATOR_H_
#define MODULES_PLANNING_TUNING_AUTOTUNING_RAW_FEATURE_GENERATOR_H_

#include <vector>
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/common/status/status.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/reference_line_info.h"
#include "modules/planning/proto/auto_tuning_raw_feature.pb.h"
namespace apollo {
namespace planning {

class AutotuningRawFeatureGenerator {
 public:
  // @brief class constructor
  AutotuningRawFeatureGenerator() = default;
  explicit AutotuningRawFeatureGenerator(
      const std::vector<double>& evaluate_time);

  // @evaluation interface
  /**
   * @brief evaluate trajectory with environment, generate model trajectory
   *        features
   * @param reference line related info
   * @param frame related status
   * @return OK if the evaluation succeeds; error otherwise.
   */
  common::Status evaluate_trajectory(
      const std::vector<common::TrajectoryPoint>& trajectory,
      const ReferenceLineInfo& reference_line_info, const Frame& frame,
      autotuning::TrajectoryRawFeature* const trajectory_feature) const;

  /**
   * @brief evaluate trajectory point with environment, generate model
   * trajectory
   *        features
   * @param reference line related info
   * @param frame related status
   * @return OK if the evaluation succeeds; error otherwise.
   */
  common::Status evaluate_trajectory_point(
      const common::TrajectoryPoint& trajectory_point,
      const ReferenceLineInfo& reference_line_info, const Frame& frame,
      autotuning::TrajectoryPointRawFeature* const trajectory_point_feature)
      const;

 private:
  std::vector<double> evaluate_time_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_TUNING_AUTOTUNING_RAW_FEATURE_GENERATOR_H_
