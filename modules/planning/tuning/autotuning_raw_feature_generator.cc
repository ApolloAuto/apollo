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

#include "modules/planning/tuning/autotuning_raw_feature_generator.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

AutotuningRawFeatureGenerator::AutotuningRawFeatureGenerator(
    const std::vector<double>& evaluate_time) {
  evaluate_time_ = evaluate_time;
}

common::Status AutotuningRawFeatureGenerator::EvaluateTrajectory(
    const std::vector<common::TrajectoryPoint>& trajectory,
    const ReferenceLineInfo& reference_line_info, const Frame& frame,
    autotuning::TrajectoryRawFeature* const trajectory_feature) const {
  return common::Status::OK();
}

common::Status AutotuningRawFeatureGenerator::EvaluateTrajectoryPoint(
    const common::TrajectoryPoint& trajectory_point,
    const ReferenceLineInfo& reference_line_info, const Frame& frame,
    autotuning::TrajectoryPointRawFeature* const trajectory_point_feature)
    const {
  return common::Status::OK();
}

common::Status AutotuningRawFeatureGenerator::EvaluateSpeedPoint(
    const common::SpeedPoint& speed_point,
    const ReferenceLineInfo& reference_line_info, const Frame& frame,
    autotuning::TrajectoryPointRawFeature* const trajectory_point_feature)
    const {
  return common::Status::OK();
}

common::Status AutotuningRawFeatureGenerator::EvaluateSpeedProfile(
    const std::vector<common::SpeedPoint>& speed_profile,
    const ReferenceLineInfo& reference_line_info, const Frame& frame,
    autotuning::TrajectoryRawFeature* const trajectory_feature) const {
  return common::Status::OK();
}

void AutotuningRawFeatureGenerator::GenerateSTBoundaries(
    const ReferenceLineInfo& reference_line_info,
    std::vector<const StBoundary*>* const boundaries) const {
  const auto& path_decision = reference_line_info.path_decision();
  const auto& adc_sl_boundary = reference_line_info.AdcSlBoundary();
  for (auto* obstacle : path_decision.path_obstacles().Items()) {
    auto id = obstacle->Id();
    if (!obstacle->st_boundary().IsEmpty()) {
      boundaries->push_back(&obstacle->st_boundary());
    } else if (FLAGS_enable_side_vehicle_st_boundary &&
               (adc_sl_boundary.start_l() > 2.0 ||
                adc_sl_boundary.end_l() < -2.0)) {
      if (path_decision.Find(id)->reference_line_st_boundary().IsEmpty()) {
        continue;
      }
      ADEBUG << "obstacle " << id << " is NOT blocking.";
      auto st_boundary_copy =
          path_decision.Find(id)->reference_line_st_boundary();
      auto st_boundary = st_boundary_copy.CutOffByT(3.5);
      if (!st_boundary.IsEmpty()) {
        auto decision = obstacle->LongitudinalDecision();
        if (decision.has_yield()) {
          st_boundary.SetBoundaryType(StBoundary::BoundaryType::YIELD);
        } else if (decision.has_overtake()) {
          st_boundary.SetBoundaryType(StBoundary::BoundaryType::OVERTAKE);
        } else if (decision.has_follow()) {
          st_boundary.SetBoundaryType(StBoundary::BoundaryType::FOLLOW);
        } else if (decision.has_stop()) {
          st_boundary.SetBoundaryType(StBoundary::BoundaryType::STOP);
        }
        st_boundary.SetId(st_boundary_copy.id());
        st_boundary.SetCharacteristicLength(
            st_boundary_copy.characteristic_length());
        boundaries->push_back(&obstacle->st_boundary());
      }
    }
  }
}
}  // namespace planning
}  // namespace apollo
