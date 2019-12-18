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

#include "modules/planning/tuning/speed_model/autotuning_speed_feature_builder.h"

#include <string>

#include "cyber/common/log.h"
#include "modules/common/proto/error_code.pb.h"

using apollo::common::ErrorCode;
using apollo::common::Status;

namespace apollo {
namespace planning {

Status AutotuningSpeedFeatureBuilder::BuildFeature(
    const autotuning::TrajectoryRawFeature& raw_feature,
    autotuning::TrajectoryFeature* const input_feature) const {
  if (input_feature == nullptr) {
    const std::string msg = "input trajectory feature is empty";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // number of input trajectory point
  int n = input_feature->point_feature_size();
  if (n != raw_feature.point_feature_size()) {
    std::ostringstream oss("raw and input feature size mismatch");
    oss << "raw : " << raw_feature.point_feature_size() << "input : " << n;
    const std::string msg = oss.str();
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  for (int i = 0; i < n; ++i) {
    const auto& raw_point_feature = raw_feature.point_feature(i);
    auto* point_feature = input_feature->mutable_point_feature(i);
    auto status = BuildPointFeature(raw_point_feature, point_feature);
    if (status != Status::OK()) {
      const std::string msg = "pointwise feature adding incorrect";
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
  }
  return Status::OK();
}

Status AutotuningSpeedFeatureBuilder::BuildPointFeature(
    const autotuning::TrajectoryPointRawFeature& raw_point_feature,
    autotuning::TrajectoryPointwiseFeature* const point_feature) const {
  // set up basic feature:
  double v = raw_point_feature.speed_feature().v();
  auto* speed_feature = point_feature->mutable_speed_input_feature();
  speed_feature->set_s(raw_point_feature.speed_feature().s());
  speed_feature->set_t(raw_point_feature.speed_feature().t());
  speed_feature->set_v(v);
  speed_feature->set_acc(raw_point_feature.speed_feature().a());
  speed_feature->set_jerk(raw_point_feature.speed_feature().j());
  speed_feature->set_speed_limit(
      raw_point_feature.speed_feature().speed_limit());

  double kappa = raw_point_feature.path_feature().cartesian_coord().kappa();
  double lateral_acc = v * v * kappa;

  speed_feature->set_path_curvature_abs(std::fabs(kappa));
  speed_feature->set_lateral_acc(lateral_acc);

  // boundary feature, virtual obstacle , e.g. routing end
  for (int i = 0; i < raw_point_feature.speed_feature().virtual_decision_size();
       ++i) {
    const auto& obj_decision =
        raw_point_feature.speed_feature().virtual_decision(i);
    auto* input_decision = speed_feature->add_virtual_obs_feature();
    map_obstacle_feature(obj_decision, input_decision);
  }

  // boundary feature, stop obstacle, based on stop decision
  for (int i = 0; i < raw_point_feature.speed_feature().stop_size(); ++i) {
    const auto& obj_decision = raw_point_feature.speed_feature().stop(i);
    auto* input_decision = speed_feature->add_stop_obs_feature();
    map_obstacle_feature(obj_decision, input_decision);
  }

  // boundary feature, overtake obstacle, based on overtake obstacle decision
  for (int i = 0; i < raw_point_feature.speed_feature().overtake_size(); ++i) {
    const auto& obj_decision = raw_point_feature.speed_feature().overtake(i);
    auto* input_decision = speed_feature->add_overtake_obs_feature();
    map_obstacle_feature(obj_decision, input_decision);
  }

  // boundary feature, yield/follow obstacle, based on yield/follow obstacle
  // decision
  for (int i = 0; i < raw_point_feature.speed_feature().follow_size(); ++i) {
    const auto& obj_decision = raw_point_feature.speed_feature().follow(i);
    auto* input_decision = speed_feature->add_follow_obs_feature();
    map_obstacle_feature(obj_decision, input_decision);
  }

  // boundary feature, nudge obstacle feature
  for (int i = 0; i < raw_point_feature.speed_feature().nudge_size(); ++i) {
    const auto& obj_decision = raw_point_feature.speed_feature().nudge(i);
    auto* input_decision = speed_feature->add_nudge_obs_feature();
    map_nudge_obs_feature(obj_decision, input_decision);
  }

  // boundary feature, side pass feature
  for (int i = 0; i < raw_point_feature.speed_feature().sidepass_front_size();
       ++i) {
    const auto& obj_decision =
        raw_point_feature.speed_feature().sidepass_front(i);
    auto* input_decision = speed_feature->add_sidepass_front_obs_feature();
    map_sidepass_obs_feature(obj_decision, input_decision);
  }
  for (int i = 0; i < raw_point_feature.speed_feature().sidepass_rear_size();
       ++i) {
    const auto& obj_decision =
        raw_point_feature.speed_feature().sidepass_rear(i);
    auto* input_decision = speed_feature->add_sidepass_rear_obs_feature();
    map_sidepass_obs_feature(obj_decision, input_decision);
  }
  return Status::OK();
}

void AutotuningSpeedFeatureBuilder::map_obstacle_feature(
    const autotuning::SpeedPointRawFeature_ObjectDecisionFeature&
        obj_raw_feature,
    autotuning::SpeedPointwiseFeature_ObstacleFeature* const input_feature)
    const {
  input_feature->set_longitudinal_distance(
      std::fabs(obj_raw_feature.relative_s()));
  input_feature->set_obstacle_speed(obj_raw_feature.speed());
  input_feature->set_relative_v(obj_raw_feature.relative_v());
  input_feature->set_lateral_distance(std::fabs(obj_raw_feature.relative_l()));
}

/**
 * @brief: map nudge obstacle to model input feature
 */
void AutotuningSpeedFeatureBuilder::map_nudge_obs_feature(
    const autotuning::SpeedPointRawFeature_ObjectDecisionFeature&
        obj_raw_feature,
    autotuning::SpeedPointwiseFeature_ObstacleFeature* const input_feature)
    const {
  input_feature->set_longitudinal_distance(
      std::fabs(obj_raw_feature.relative_s()));
  input_feature->set_obstacle_speed(obj_raw_feature.speed());
  input_feature->set_relative_v(obj_raw_feature.relative_v());
  input_feature->set_lateral_distance(std::fabs(obj_raw_feature.relative_l()));
}

/**
 * @brief: map sidepass obstacle to model input feature
 */
void AutotuningSpeedFeatureBuilder::map_sidepass_obs_feature(
    const autotuning::SpeedPointRawFeature_ObjectDecisionFeature&
        obj_raw_feature,
    autotuning::SpeedPointwiseFeature_ObstacleFeature* const input_feature)
    const {
  input_feature->set_longitudinal_distance(
      std::fabs(obj_raw_feature.relative_s()));
  input_feature->set_obstacle_speed(obj_raw_feature.speed());
  input_feature->set_relative_v(obj_raw_feature.relative_v());
  input_feature->set_lateral_distance(std::fabs(obj_raw_feature.relative_l()));
}
}  // namespace planning
}  // namespace apollo
