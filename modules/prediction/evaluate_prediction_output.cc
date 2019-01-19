/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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
 * @brief Generate trajectory-point-level evaluation results
 * @param input_feature_proto_file: Feature proto with true future status
 */
#include "cyber/common/file.h"

#include "modules/common/adapters/proto/adapter_config.pb.h"
#include "modules/prediction/common/prediction_system_gflags.h"
#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/proto/feature.pb.h"
#include "modules/prediction/proto/offline_features.pb.h"
#include "modules/prediction/proto/prediction_conf.pb.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"
#include "modules/prediction/proto/prediction_output_evaluation.pb.h"
#include "modules/prediction/container/container_manager.h"
#include "modules/prediction/container/obstacles/obstacles_container.h"
#include "modules/prediction/evaluator/evaluator_manager.h"
#include "modules/prediction/predictor/predictor_manager.h"

namespace apollo {
namespace prediction {

void Initialize() {
  PredictionConf prediction_conf;
  if (!apollo::common::util::GetProtoFromFile(
           FLAGS_prediction_conf_file, &prediction_conf)) {
    AERROR << "Unable to load prediction conf file: "
           << FLAGS_prediction_conf_file;
    return;
  }
  ADEBUG << "Prediction config file is loaded into: "
            << prediction_conf.ShortDebugString();

  apollo::common::adapter::AdapterManagerConfig adapter_conf;
  if (!apollo::common::util::GetProtoFromFile(
    FLAGS_prediction_adapter_config_filename, &adapter_conf)) {
    AERROR << "Unable to load adapter conf file: "
           << FLAGS_prediction_adapter_config_filename;
    return;
  }
  // Initialization of all managers
  ContainerManager::Instance()->Init(adapter_conf);
  EvaluatorManager::Instance()->Init(prediction_conf);
  PredictorManager::Instance()->Init(prediction_conf);
}

bool IsCorrectlyPredictedOnLane(
    const PredictionTrajectoryPoint& future_point,
    const PredictionObstacle& prediction_obstacle) {
  double future_relative_time = future_point.timestamp() -
                                prediction_obstacle.timestamp();
  const auto& predicted_trajectories = prediction_obstacle.trajectory();
  for (const auto& predicted_traj : predicted_trajectories) {
    // find an index, TODO(kechxu) use binary search to speed up
    int i = 0;
    while (i + 1 < predicted_traj.trajectory_point_size() &&
           predicted_traj.trajectory_point(i + 1).relative_time() <
           future_relative_time) {
      ++i;
    }
    // TODO(kechxu) consider interpolation
    double predicted_x = predicted_traj.trajectory_point(i).path_point().x();
    double predicted_y = predicted_traj.trajectory_point(i).path_point().y();
    double diff_x = std::abs(predicted_x - future_point.path_point().x());
    double diff_y = std::abs(predicted_y - future_point.path_point().y());
    if (diff_x < FLAGS_distance_threshold_on_lane &&
        diff_y < FLAGS_distance_threshold_on_lane) {
      return true;
    }
  }
  return false;
}

bool IsCorrectlyPredictedJunction(
    const PredictionTrajectoryPoint& future_point,
    const PredictionObstacle& prediction_obstacle) {
  // TODO(kechxu) implement
  return false;
}

double CorrectlyPredictedPortionOnLane(
    Obstacle* obstacle_ptr, const double time_range,
    int* num_predicted_trajectory) {
  PredictionObstacle prediction_obstacle;
  EvaluatorManager::Instance()->EvaluateObstacle(obstacle_ptr);
  PredictorManager::Instance()->PredictObstacle(
      obstacle_ptr, &prediction_obstacle, nullptr);
  const Feature& feature = obstacle_ptr->latest_feature();
  int total_count = 0;
  int correct_count = 0;
  for (const auto& future_point : feature.future_trajectory_points()) {
    if (IsCorrectlyPredictedOnLane(future_point, prediction_obstacle)) {
      ++correct_count;
    }
    ++total_count;
  }
  if (total_count == 0) {
    return 0.0;
  }
  return static_cast<double>(correct_count) / static_cast<double>(total_count);
}

double CorrectlyPredictedPortionJunction(
    Obstacle* obstacle_ptr, const double time_range,
    int* num_predicted_trajectory) {
  PredictionObstacle prediction_obstacle;
  EvaluatorManager::Instance()->EvaluateObstacle(obstacle_ptr);
  PredictorManager::Instance()->PredictObstacle(
      obstacle_ptr, &prediction_obstacle, nullptr);
  const Feature& feature = obstacle_ptr->latest_feature();
  int total_count = 0;
  int correct_count = 0;
  for (const auto& future_point : feature.future_trajectory_points()) {
    if (IsCorrectlyPredictedJunction(future_point, prediction_obstacle)) {
      ++correct_count;
    }
    ++total_count;
  }
  if (total_count == 0) {
    return 0.0;
  }
  return static_cast<double>(correct_count) / static_cast<double>(total_count);
}

void UpdateMetrics(const int num_trajectory, const double correct_portion,
    TrajectoryEvaluationMetrics* const metrics) {
  metrics->set_num_frame_obstacle(metrics->num_frame_obstacle() + 1);
  metrics->set_num_predicted_trajectory(
      metrics->num_predicted_trajectory() + num_trajectory);
  metrics->set_num_correctly_predicted_frame_obstacle(
      metrics->num_correctly_predicted_frame_obstacle() + correct_portion);
}

void ComputeResults(TrajectoryEvaluationMetrics* const metrics) {
  if (metrics->num_frame_obstacle() == 0 ||
      metrics->num_predicted_trajectory() == 0) {
    return;
  }
  metrics->set_recall(metrics->num_correctly_predicted_frame_obstacle() /
                     metrics->num_frame_obstacle());
  metrics->set_recall(metrics->num_correctly_predicted_frame_obstacle() /
                     metrics->num_predicted_trajectory());
  // TODO(kechxu) compute sum_squared_error
}

TrajectoryEvaluationMetricsGroup Evaluate(
    const Features& features_with_future_status) {
  TrajectoryEvaluationMetrics on_lane_metrics;
  TrajectoryEvaluationMetrics junction_metrics;
  TrajectoryEvaluationMetricsGroup metrics_group;
  auto obstacles_container_ptr =
      ContainerManager::Instance()->GetContainer<ObstaclesContainer>(
        apollo::common::adapter::AdapterConfig::PERCEPTION_OBSTACLES);
  for (const Feature& feature : features_with_future_status.feature()) {
    obstacles_container_ptr->InsertFeatureProto(feature);
    Obstacle* obstacle_ptr = obstacles_container_ptr->GetObstacle(feature.id());
    if (obstacle_ptr->HasJunctionFeatureWithExits()) {
      int num_trajectory = 0;
      double correct_portion = CorrectlyPredictedPortionJunction(
          obstacle_ptr, 3.0, &num_trajectory);
      UpdateMetrics(num_trajectory, correct_portion, &junction_metrics);
    } else if (obstacle_ptr->IsOnLane()) {
      int num_trajectory = 0;
      double correct_portion = CorrectlyPredictedPortionOnLane(
          obstacle_ptr, 3.0, &num_trajectory);
      UpdateMetrics(num_trajectory, correct_portion, &on_lane_metrics);
    }
  }
  ComputeResults(&junction_metrics);
  ComputeResults(&on_lane_metrics);
  metrics_group.mutable_junction_metrics()->CopyFrom(junction_metrics);
  metrics_group.mutable_on_lane_metrics()->CopyFrom(on_lane_metrics);
  return metrics_group;
}

}  // namespace prediction
}  // namespace apollo

int main(int argc, char *argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_enable_trim_prediction_trajectory = false;
  apollo::prediction::Initialize();
  // TODO(kechxu) Load feature proto and run Evaluate
}
