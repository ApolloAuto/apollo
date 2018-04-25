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

#include "modules/prediction/prediction.h"

#include <cmath>

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/time/time.h"
#include "modules/common/util/file.h"
#include "modules/prediction/common/feature_output.h"
#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/common/prediction_map.h"
#include "modules/prediction/common/validation_checker.h"
#include "modules/prediction/container/container_manager.h"
#include "modules/prediction/container/obstacles/obstacles_container.h"
#include "modules/prediction/container/pose/pose_container.h"
#include "modules/prediction/evaluator/evaluator_manager.h"
#include "modules/prediction/predictor/predictor_manager.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"

namespace apollo {
namespace prediction {

using ::apollo::common::ErrorCode;
using ::apollo::common::Status;
using ::apollo::common::TrajectoryPoint;
using ::apollo::common::adapter::AdapterConfig;
using ::apollo::common::adapter::AdapterManager;
using ::apollo::common::math::Vec2d;
using ::apollo::common::time::Clock;
using ::apollo::localization::LocalizationEstimate;
using ::apollo::perception::PerceptionObstacle;
using ::apollo::perception::PerceptionObstacles;

std::string Prediction::Name() const { return FLAGS_prediction_module_name; }

Status Prediction::Init() {
  start_time_ = Clock::NowInSeconds();

  // Load prediction conf
  prediction_conf_.Clear();
  if (!common::util::GetProtoFromFile(FLAGS_prediction_conf_file,
                                      &prediction_conf_)) {
    return OnError("Unable to load prediction conf file: " +
                   FLAGS_prediction_conf_file);
  } else {
    ADEBUG << "Prediction config file is loaded into: "
           << prediction_conf_.ShortDebugString();
  }

  adapter_conf_.Clear();
  if (!common::util::GetProtoFromFile(FLAGS_prediction_adapter_config_filename,
                                      &adapter_conf_)) {
    return OnError("Unable to load adapter conf file: " +
                   FLAGS_prediction_adapter_config_filename);
  } else {
    ADEBUG << "Adapter config file is loaded into: "
           << adapter_conf_.ShortDebugString();
  }

  // Initialization of all managers
  AdapterManager::Init(adapter_conf_);
  ContainerManager::instance()->Init(adapter_conf_);
  EvaluatorManager::instance()->Init(prediction_conf_);
  PredictorManager::instance()->Init(prediction_conf_);

  CHECK(AdapterManager::GetLocalization()) << "Localization is not registered.";
  CHECK(AdapterManager::GetPerceptionObstacles())
      << "Perception is not registered.";

  // Set localization callback function
  AdapterManager::AddLocalizationCallback(&Prediction::OnLocalization, this);
  // Set planning callback function
  AdapterManager::AddPlanningCallback(&Prediction::OnPlanning, this);
  // Set perception obstacle callback function
  AdapterManager::AddPerceptionObstaclesCallback(&Prediction::RunOnce, this);

  if (!FLAGS_use_navigation_mode && !PredictionMap::Ready()) {
    return OnError("Map cannot be loaded.");
  }

  if (FLAGS_prediction_offline_mode) {
    if (!FeatureOutput::Ready()) {
      return OnError("Feature output is not ready.");
    }
  }

  return Status::OK();
}

Status Prediction::Start() { return Status::OK(); }

void Prediction::Stop() {
  if (FLAGS_prediction_offline_mode) {
    FeatureOutput::Close();
  }
}

void Prediction::OnLocalization(const LocalizationEstimate& localization) {
  PoseContainer* pose_container = dynamic_cast<PoseContainer*>(
      ContainerManager::instance()->GetContainer(AdapterConfig::LOCALIZATION));
  CHECK_NOTNULL(pose_container);
  pose_container->Insert(localization);

  ADEBUG << "Received a localization message ["
         << localization.ShortDebugString() << "].";
}

void Prediction::OnPlanning(const planning::ADCTrajectory& adc_trajectory) {
  ADCTrajectoryContainer* adc_trajectory_container =
      dynamic_cast<ADCTrajectoryContainer*>(
          ContainerManager::instance()->GetContainer(
              AdapterConfig::PLANNING_TRAJECTORY));
  CHECK_NOTNULL(adc_trajectory_container);
  adc_trajectory_container->Insert(adc_trajectory);

  ADEBUG << "Received a planning message [" << adc_trajectory.ShortDebugString()
         << "].";
}

void Prediction::RunOnce(const PerceptionObstacles& perception_obstacles) {
  if (FLAGS_prediction_test_mode && FLAGS_prediction_test_duration > 0 &&
      (Clock::NowInSeconds() - start_time_ > FLAGS_prediction_test_duration)) {
    AINFO << "Prediction finished running in test mode";
    ros::shutdown();
  }

  // Update relative map if needed
  AdapterManager::Observe();
  if (FLAGS_use_navigation_mode && !PredictionMap::Ready()) {
    AERROR << "Relative map is empty.";
    return;
  }

  double start_timestamp = Clock::NowInSeconds();

  // Insert obstacle
  ObstaclesContainer* obstacles_container = dynamic_cast<ObstaclesContainer*>(
      ContainerManager::instance()->GetContainer(
          AdapterConfig::PERCEPTION_OBSTACLES));
  CHECK_NOTNULL(obstacles_container);
  obstacles_container->Insert(perception_obstacles);

  ADEBUG << "Received a perception message ["
         << perception_obstacles.ShortDebugString() << "].";

  // Update ADC status
  PoseContainer* pose_container = dynamic_cast<PoseContainer*>(
      ContainerManager::instance()->GetContainer(AdapterConfig::LOCALIZATION));
  ADCTrajectoryContainer* adc_container = dynamic_cast<ADCTrajectoryContainer*>(
      ContainerManager::instance()->GetContainer(
          AdapterConfig::PLANNING_TRAJECTORY));
  CHECK_NOTNULL(pose_container);
  CHECK_NOTNULL(adc_container);

  PerceptionObstacle* adc = pose_container->ToPerceptionObstacle();
  if (adc != nullptr) {
    obstacles_container->InsertPerceptionObstacle(*adc, adc->timestamp());
    double x = adc->position().x();
    double y = adc->position().y();
    ADEBUG << "Get ADC position [" << std::fixed << std::setprecision(6) << x
           << ", " << std::fixed << std::setprecision(6) << y << "].";
    Vec2d adc_position(x, y);
    adc_container->SetPosition(adc_position);
  }

  // Make evaluations
  EvaluatorManager::instance()->Run(perception_obstacles);

  // No prediction for offline mode
  if (FLAGS_prediction_offline_mode) {
    return;
  }

  // Make predictions
  PredictorManager::instance()->Run(perception_obstacles);

  auto prediction_obstacles =
      PredictorManager::instance()->prediction_obstacles();
  prediction_obstacles.set_start_timestamp(start_timestamp);
  prediction_obstacles.set_end_timestamp(Clock::NowInSeconds());

  if (FLAGS_prediction_test_mode) {
    for (auto const& prediction_obstacle :
         prediction_obstacles.prediction_obstacle()) {
      for (auto const& trajectory : prediction_obstacle.trajectory()) {
        for (auto const& trajectory_point : trajectory.trajectory_point()) {
          if (!ValidationChecker::ValidTrajectoryPoint(trajectory_point)) {
            AERROR << "Invalid trajectory point ["
                   << trajectory_point.ShortDebugString() << "]";
            return;
          }
        }
      }
    }
  }

  Publish(&prediction_obstacles);
}

Status Prediction::OnError(const std::string& error_msg) {
  return Status(ErrorCode::PREDICTION_ERROR, error_msg);
}

}  // namespace prediction
}  // namespace apollo
