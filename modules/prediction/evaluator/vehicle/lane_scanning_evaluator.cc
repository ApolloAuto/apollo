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

#include <limits>
#include <utility>

#include "modules/common/util/file.h"
#include "modules/prediction/common/feature_output.h"
#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/common/prediction_system_gflags.h"
#include "modules/prediction/container/container_manager.h"
#include "modules/prediction/container/obstacles/obstacles_container.h"
#include "modules/prediction/evaluator/vehicle/lane_scanning_evaluator.h"

namespace apollo {
namespace prediction {

using apollo::common::adapter::AdapterConfig;
using apollo::common::util::GetProtoFromFile;

// Helper function for converting world coordinate of a point
// to relative coordinate with respect to the object (obstacle or ADC).
std::pair<double, double> WorldCoordToObjCoord
    (std::pair<double, double> input_world_coord,
     std::pair<double, double> obj_world_coord,
     double obj_world_angle) {
  double x_diff = input_world_coord.first - obj_world_coord.first;
  double y_diff = input_world_coord.second - obj_world_coord.second;
  double rho = std::sqrt(x_diff * x_diff + y_diff * y_diff);
  double theta = std::atan2(y_diff, x_diff) - obj_world_angle;

  return std::make_pair(std::cos(theta)*rho, std::sin(theta)*rho);
}

double WorldAngleToObjAngle(double input_world_angle,
                            double obj_world_angle) {
  return common::math::NormalizeAngle(input_world_angle - obj_world_angle);
}

LaneScanningEvaluator::LaneScanningEvaluator() {
}

void LaneScanningEvaluator::Evaluate(Obstacle* obstacle_ptr) {
  // Sanity checks.
  CHECK_NOTNULL(obstacle_ptr);
  int id = obstacle_ptr->id();
  if (!obstacle_ptr->latest_feature().IsInitialized()) {
    AERROR << "Obstacle [" << id << "] has no latest feature.";
    return;
  }
  Feature* latest_feature_ptr = obstacle_ptr->mutable_latest_feature();
  CHECK_NOTNULL(latest_feature_ptr);
  if (!latest_feature_ptr->has_lane() ||
      !latest_feature_ptr->lane().has_lane_graph_ordered()) {
    AERROR << "Obstacle [" << id << "] has no lane graph.";
    return;
  }
  LaneGraph* lane_graph_ptr =
      latest_feature_ptr->mutable_lane()->mutable_lane_graph_ordered();
  CHECK_NOTNULL(lane_graph_ptr);
  if (lane_graph_ptr->lane_sequence_size() == 0) {
    AERROR << "Obstacle [" << id << "] has no lane sequences.";
    return;
  }
  ADEBUG << "There are " << lane_graph_ptr->lane_sequence_size()
         << " lane sequences to scan.";

  // Extract features, and:
  //  - if in offline mode, save it locally for training.
  //  - if in online mode, pass it through trained model to evaluate.
  std::vector<double> feature_values;
  ExtractFeatures(obstacle_ptr, lane_graph_ptr, &feature_values);
  // TODO(jiacheng): sanity check of extracted feature_values:
  if (FLAGS_prediction_offline_mode) {
    // TODO(jiacheng): save the extracted features locally for offline training.
  } else {
    // TODO(jiacheng): once the model is trained, implement this online part.
  }
}

bool LaneScanningEvaluator::ExtractFeatures(
    const Obstacle* obstacle_ptr, const LaneGraph* lane_graph_ptr,
    std::vector<double>* feature_values) {
  // Sanity checks.
  CHECK_NOTNULL(obstacle_ptr);
  int id = obstacle_ptr->id();
  CHECK_NOTNULL(lane_graph_ptr);

  // Extract obstacle related features.
  std::vector<double> obstacle_feature_values;
  if (!ExtractObstacleFeatures(obstacle_ptr, &obstacle_feature_values)) {
    ADEBUG << "Failed to extract obstacle features for obs_id = " << id;
  }
  if (obstacle_feature_values.size() != OBSTACLE_FEATURE_SIZE) {
    ADEBUG << "Obstacle [" << id << "] has fewer than "
           << "expected obstacle feature_values "
           << obstacle_feature_values.size() << ".";
    return false;
  }
  ADEBUG << "Obstacle feature size = " << obstacle_feature_values.size();
  feature_values->insert(feature_values->end(),
                         obstacle_feature_values.begin(),
                         obstacle_feature_values.end());

  // Extract static environmental (lane-related) features.
  std::vector<double> static_feature_values;
  if (!ExtractStaticEnvFeatures(lane_graph_ptr, &static_feature_values)) {
    ADEBUG << "Failed to extract static environmental features around obs_id = "
           << id;
  }
  // TODO(jiacheng): sanity check of extracted static env features:
  feature_values->insert(feature_values->end(),
                         static_feature_values.begin(),
                         static_feature_values.end());
  return true;
}

bool LaneScanningEvaluator::ExtractObstacleFeatures(
    const Obstacle* obstacle_ptr, std::vector<double>* feature_values) {
  return true;
}

bool LaneScanningEvaluator::ExtractStaticEnvFeatures(
    const LaneGraph* lane_graph_ptr, std::vector<double>* feature_values) {
  return true;
}

}  // namespace prediction
}  // namespace apollo
