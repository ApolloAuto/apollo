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
#include "modules/planning/planning_base/learning_based/tuning/speed_model/autotuning_speed_mlp_model.h"

#include "modules/planning/planning_base/learning_based/tuning/speed_model/autotuning_speed_feature_builder.h"

namespace apollo {
namespace planning {

/**
 * @brief: max considerred obstacle range
 */
namespace {
constexpr double kMaxFollow = 100.0;
constexpr double kMaxOvertake = 100.0;
constexpr double kMaxStop = 60.0;
constexpr double kMaxNudge = 60.0;
constexpr double kMaxNudgeLateralDistance = 10.0;
constexpr double kMaxSidePassDistance = 100.0;
}  // namespace

common::Status AutotuningSpeedMLPModel::SetParams() {
  mlp_model_.reset(new AutotuningMLPModel());
  feature_builder_.reset(new AutotuningSpeedFeatureBuilder());
  return common::Status::OK();
}

double AutotuningSpeedMLPModel::Evaluate(
    const autotuning::TrajectoryFeature& trajectory_feature) const {
  return 0.0;
}

double AutotuningSpeedMLPModel::Evaluate(
    const autotuning::TrajectoryPointwiseFeature& point_feature) const {
  return 0.0;
}

void AutotuningSpeedMLPModel::FlattenFeatures(
    const autotuning::TrajectoryFeature& feature,
    Eigen::MatrixXd* const flat_feature) const {
  int row_count = feature.point_feature_size();
  int col_count = 21;
  flat_feature->resize(row_count, col_count);

  for (int i = 0; i < row_count; ++i) {
    FlattenFeatures(feature.point_feature(i).speed_input_feature(), i,
                    flat_feature);
  }
}

void AutotuningSpeedMLPModel::FlattenFeatures(
    const autotuning::SpeedPointwiseFeature& speed_point_feature, const int row,
    Eigen::MatrixXd* const flat_feature) const {
  double v = speed_point_feature.v();
  double acc = speed_point_feature.acc();
  double jerk = speed_point_feature.jerk();
  double speed_limit = speed_point_feature.speed_limit();

  double lateral_acc = speed_point_feature.lateral_acc();
  // has collision , collision = 1, else 0
  double collision = 0.0;
  // obstacle feature, looping to find among all obstacle
  double follow_distance = kMaxFollow;
  double follow_v_rel = 0.0;
  double overtake_distance = kMaxOvertake;
  double overtake_v_rel = 0.0;
  double stop_distance = kMaxStop;
  double stop_v_rel = 0.0;
  double virtual_distance = kMaxStop;
  double virtual_v_rel = 0.0;
  double nudge_distance = kMaxNudge;
  double nudge_v_rel = 0.0;
  double nudge_lateral_distance = kMaxNudgeLateralDistance;
  double sidepass_front_v_rel = 0.0;
  double sidepass_front_distance = kMaxSidePassDistance;
  double sidepass_rear_v_rel = 0.0;
  double sidepass_rear_distance = kMaxSidePassDistance;

  // looping obstacles to find the value minimize the distance
  for (int i = 0; i < speed_point_feature.follow_obs_feature_size(); ++i) {
    const auto& follow_obs_feature = speed_point_feature.follow_obs_feature(i);
    if (follow_distance > follow_obs_feature.longitudinal_distance()) {
      follow_distance = follow_obs_feature.longitudinal_distance();
      follow_v_rel = follow_obs_feature.relative_v();
    }
  }
  // looping obstacles to find the value minimize the distance
  for (int i = 0; i < speed_point_feature.overtake_obs_feature_size(); ++i) {
    const auto& overtake_obs_feature =
        speed_point_feature.overtake_obs_feature(i);
    if (overtake_distance > overtake_obs_feature.longitudinal_distance()) {
      overtake_distance = overtake_obs_feature.longitudinal_distance();
      overtake_v_rel = overtake_obs_feature.relative_v();
    }
  }
  // looping obstacles to find the value minimize the distance
  for (int i = 0; i < speed_point_feature.stop_obs_feature_size(); ++i) {
    const auto& stop_obs_feature = speed_point_feature.stop_obs_feature(i);
    if (stop_distance > stop_obs_feature.longitudinal_distance()) {
      stop_distance = stop_obs_feature.longitudinal_distance();
      stop_v_rel = stop_obs_feature.relative_v();
    }
  }
  // looping obstacles to find the value minimize the distance
  for (int i = 0; i < speed_point_feature.virtual_obs_feature_size(); ++i) {
    const auto& virtual_obs_feature =
        speed_point_feature.virtual_obs_feature(i);
    if (virtual_distance > virtual_obs_feature.longitudinal_distance()) {
      virtual_distance = virtual_obs_feature.longitudinal_distance();
      virtual_v_rel = virtual_obs_feature.relative_v();
    }
  }
  // looping obstacles to find the value minimize the distance
  for (int i = 0; i < speed_point_feature.nudge_obs_feature_size(); ++i) {
    const auto& nudge_obs_feature = speed_point_feature.nudge_obs_feature(i);
    if (nudge_distance > nudge_obs_feature.longitudinal_distance()) {
      nudge_distance = nudge_obs_feature.longitudinal_distance();
      nudge_v_rel = nudge_obs_feature.relative_v();
      nudge_lateral_distance = nudge_obs_feature.lateral_distance();
    }
  }
  // looping obstacles to find the value minimize the distance
  for (int i = 0; i < speed_point_feature.sidepass_front_obs_feature_size();
       ++i) {
    const auto& sidepass_front_obs_feature =
        speed_point_feature.sidepass_front_obs_feature(i);
    if (sidepass_front_distance >
        sidepass_front_obs_feature.longitudinal_distance()) {
      sidepass_front_distance =
          sidepass_front_obs_feature.longitudinal_distance();
      sidepass_front_v_rel = sidepass_front_obs_feature.relative_v();
    }
  }
  // looping obstacles to find the value minimize the distance
  for (int i = 0; i < speed_point_feature.sidepass_rear_obs_feature_size();
       ++i) {
    const auto& sidepass_rear_obs_feature =
        speed_point_feature.sidepass_rear_obs_feature(i);
    if (sidepass_rear_distance >
        sidepass_rear_obs_feature.longitudinal_distance()) {
      sidepass_rear_distance =
          sidepass_rear_obs_feature.longitudinal_distance();
      sidepass_rear_v_rel = sidepass_rear_obs_feature.relative_v();
    }
  }

  (*flat_feature)(row, 0) = v;
  (*flat_feature)(row, 1) = speed_limit;
  (*flat_feature)(row, 2) = acc;
  (*flat_feature)(row, 3) = jerk;
  (*flat_feature)(row, 4) = lateral_acc;
  (*flat_feature)(row, 5) = collision;
  (*flat_feature)(row, 6) = follow_distance;
  (*flat_feature)(row, 7) = follow_v_rel;
  (*flat_feature)(row, 8) = overtake_distance;
  (*flat_feature)(row, 9) = overtake_v_rel;
  (*flat_feature)(row, 10) = nudge_distance;
  (*flat_feature)(row, 11) = nudge_v_rel;
  (*flat_feature)(row, 12) = nudge_lateral_distance;
  (*flat_feature)(row, 13) = stop_distance;
  (*flat_feature)(row, 14) = stop_v_rel;
  (*flat_feature)(row, 15) = virtual_distance;
  (*flat_feature)(row, 16) = virtual_v_rel;
  (*flat_feature)(row, 17) = sidepass_front_distance;
  (*flat_feature)(row, 18) = sidepass_front_v_rel;
  (*flat_feature)(row, 19) = sidepass_rear_distance;
  (*flat_feature)(row, 20) = sidepass_rear_v_rel;
}

}  // namespace planning
}  // namespace apollo
