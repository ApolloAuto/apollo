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
}

void LaneScanningEvaluator::ExtractFeatures(
    const Obstacle* obstacle_ptr, std::vector<double>* feature_values) {
}

void LaneScanningEvaluator::ExtractObstacleFeatures(
    const Obstacle* obstacle_ptr, std::vector<double>* feature_values) {
}

void LaneScanningEvaluator::ExtractStaticEnvFeatures(
    const Obstacle* obstacle_ptr, std::vector<double>* feature_values) {
}

}  // namespace prediction
}  // namespace apollo
