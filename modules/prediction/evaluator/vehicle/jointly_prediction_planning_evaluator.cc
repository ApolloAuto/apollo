/******************************************************************************
 * Copyright 2021 The Apollo Authors. All Rights Reserved.
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

#include "modules/prediction/evaluator/vehicle/jointly_prediction_planning_evaluator.h"

#include <omp.h>

#include "Eigen/Dense"
#include "cyber/common/file.h"
#include "modules/common/math/linear_interpolation.h"
#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/common/prediction_map.h"
#include "modules/prediction/common/prediction_system_gflags.h"
#include "modules/prediction/common/prediction_util.h"
#include "modules/prediction/container/adc_trajectory/adc_trajectory_container.h"

namespace apollo {
namespace prediction {

using apollo::common::TrajectoryPoint;
using apollo::common::math::Vec2d;
using apollo::common::math::InterpolateUsingLinearApproximation;

JointlyPredictionPlanningEvaluator::JointlyPredictionPlanningEvaluator(
    SemanticMap* semantic_map)
    : device_(torch::kCPU), semantic_map_(semantic_map) {
  evaluator_type_ = ObstacleConf::JOINTLY_PREDICTION_PLANNING_EVALUATOR;
  LoadModel();
}

void JointlyPredictionPlanningEvaluator::Clear() {}

bool JointlyPredictionPlanningEvaluator::Evaluate(Obstacle* obstacle_ptr,
                                     ObstaclesContainer* obstacles_container) {
  const ADCTrajectoryContainer* adc_trajectory_container;
  Evaluate(adc_trajectory_container, obstacle_ptr, obstacles_container);
  return true;
}

bool JointlyPredictionPlanningEvaluator::Evaluate(
      const ADCTrajectoryContainer* adc_trajectory_container,
      Obstacle* obstacle_ptr,
      ObstaclesContainer* obstacles_container) {
  omp_set_num_threads(1);

  obstacle_ptr->SetEvaluatorType(evaluator_type_);

  Clear();
  CHECK_NOTNULL(obstacle_ptr);
  int id = obstacle_ptr->id();
  if (!obstacle_ptr->latest_feature().IsInitialized()) {
    AERROR << "Obstacle [" << id << "] has no latest feature.";
    return false;
  }
  Feature* latest_feature_ptr = obstacle_ptr->mutable_latest_feature();
  CHECK_NOTNULL(latest_feature_ptr);

  if (!FLAGS_enable_semantic_map) {
    ADEBUG << "Not enable semantic map, exit semantic_lstm_evaluator.";
    return false;
  }
  cv::Mat feature_map;
  if (!semantic_map_->GetMapById(id, &feature_map)) {
    return false;
  }
  if (adc_trajectory_container == nullptr) {
    AERROR << "Null adc trajectory container";
    return false;
  }
  // Process the feature_map
  cv::cvtColor(feature_map, feature_map, cv::COLOR_BGR2RGB);
  cv::Mat img_float;
  feature_map.convertTo(img_float, CV_32F, 1.0 / 255);
  torch::Tensor img_tensor = torch::from_blob(img_float.data, {1, 224, 224, 3});
  img_tensor = img_tensor.permute({0, 3, 1, 2});
  img_tensor[0][0] = img_tensor[0][0].sub(0.485).div(0.229);
  img_tensor[0][1] = img_tensor[0][1].sub(0.456).div(0.224);
  img_tensor[0][2] = img_tensor[0][2].sub(0.406).div(0.225);

  // Extract features of pos_history
  std::vector<std::pair<double, double>> pos_history(20, {0.0, 0.0});
  if (!ExtractObstacleHistory(obstacle_ptr, &pos_history)) {
    ADEBUG << "Obstacle [" << id << "] failed to extract obstacle history";
    return false;
  }
  // Process obstacle_history
  torch::Tensor obstacle_pos = torch::zeros({1, 20, 2});
  torch::Tensor obstacle_pos_step = torch::zeros({1, 20, 2});
  for (int i = 0; i < 20; ++i) {
    obstacle_pos[0][19 - i][0] = pos_history[i].first;
    obstacle_pos[0][19 - i][1] = pos_history[i].second;
    if (i == 19 || (i > 0 && pos_history[i].first < 1.0e-10)) {
      break;
    }
    obstacle_pos_step[0][19 - i][0] =
        pos_history[i].first - pos_history[i + 1].first;
    obstacle_pos_step[0][19 - i][1] =
        pos_history[i].second - pos_history[i + 1].second;
  }

  // Process ADC trajectory & Extract features of ADC trajectory
  std::vector<std::pair<double, double>> adc_traj_curr_pos(30, {0.0, 0.0});
  torch::Tensor adc_trajectory = torch::zeros({1, 30, 6});
  const auto& adc_traj = adc_trajectory_container->adc_trajectory();
  size_t adc_traj_points_num = adc_traj.trajectory_point().size();
  std::vector<TrajectoryPoint> adc_traj_points;
  // ADC trajectory info as model input needs to match with
  // the predicted obstalce's timestamp.
  double time_interval = obstacle_ptr->latest_feature().timestamp() -
      adc_traj.header().timestamp_sec();
  for (size_t i = 0; i < adc_traj_points_num - 1; ++i) {
    double delta_time = time_interval -
        adc_traj.trajectory_point(0).relative_time();
    adc_traj_points.emplace_back(
        InterpolateUsingLinearApproximation(
            adc_traj.trajectory_point(i),
            adc_traj.trajectory_point(i + 1), delta_time));
  }
  if (!ExtractADCTrajectory(&adc_traj_points,
      obstacle_ptr, &adc_traj_curr_pos)) {
    ADEBUG << "Failed to extract adc trajectory";
    return false;
  }
  size_t traj_points_num = adc_traj_points.size();
  for (size_t j = 0; j < 30; ++j) {
    if (j > traj_points_num - 1) {
      adc_trajectory[0][j][0] =
          adc_traj_curr_pos[traj_points_num - 1].first;
      adc_trajectory[0][j][1] =
          adc_traj_curr_pos[traj_points_num - 1].second;
      adc_trajectory[0][j][2] =
          adc_traj_points[traj_points_num - 1].path_point().theta();
      adc_trajectory[0][j][3] =
          adc_traj_points[traj_points_num - 1].v();
      adc_trajectory[0][j][4] =
          adc_traj_points[traj_points_num - 1].a();
      adc_trajectory[0][j][5] =
          adc_traj_points[traj_points_num - 1].path_point().kappa();
    } else {
      adc_trajectory[0][j][0] =
          adc_traj_curr_pos[j].first;
      adc_trajectory[0][j][1] =
          adc_traj_curr_pos[j].second;
      adc_trajectory[0][j][2] =
          adc_traj_points[j].path_point().theta();
      adc_trajectory[0][j][3] =
          adc_traj_points[j].v();
      adc_trajectory[0][j][4] =
          adc_traj_points[j].a();
      adc_trajectory[0][j][5] =
          adc_traj_points[j].path_point().kappa();
    }
  }

  // Build input features for torch
  std::vector<torch::jit::IValue> torch_inputs;

  auto X_value = c10::ivalue::Tuple::create(
      {std::move(img_tensor.to(device_)), std::move(obstacle_pos.to(device_)),
       std::move(obstacle_pos_step.to(device_))});
  torch_inputs.push_back(c10::ivalue::Tuple::create(
      {X_value, std::move(adc_trajectory.to(device_))}));

  // Compute pred_traj
  std::vector<double> pred_traj;

  auto start_time = std::chrono::system_clock::now();
  at::Tensor torch_output_tensor = torch_default_output_tensor_;
  torch_output_tensor =
      torch_vehicle_model_.forward(torch_inputs).toTensor().to(torch::kCPU);

  auto end_time = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = end_time - start_time;
  ADEBUG << "Semantic_LSTM_evaluator used time: " << diff.count() * 1000
         << " ms.";
  auto torch_output = torch_output_tensor.accessor<float, 3>();

  // Get the trajectory
  double pos_x = latest_feature_ptr->position().x();
  double pos_y = latest_feature_ptr->position().y();
  Trajectory* trajectory = latest_feature_ptr->add_predicted_trajectory();
  trajectory->set_probability(1.0);

  for (int i = 0; i < 30; ++i) {
    double prev_x = pos_x;
    double prev_y = pos_y;
    if (i > 0) {
      const auto& last_point = trajectory->trajectory_point(i - 1).path_point();
      prev_x = last_point.x();
      prev_y = last_point.y();
    }
    TrajectoryPoint* point = trajectory->add_trajectory_point();
    double dx = static_cast<double>(torch_output[0][i][0]);
    double dy = static_cast<double>(torch_output[0][i][1]);

    double heading = latest_feature_ptr->velocity_heading();
    Vec2d offset(dx, dy);
    Vec2d rotated_offset = offset.rotate(heading);
    double point_x = pos_x + rotated_offset.x();
    double point_y = pos_y + rotated_offset.y();
    point->mutable_path_point()->set_x(point_x);
    point->mutable_path_point()->set_y(point_y);

    if (i < 10) {  // use origin heading for the first second
      point->mutable_path_point()->set_theta(
          latest_feature_ptr->velocity_heading());
    } else {
      point->mutable_path_point()->set_theta(
          std::atan2(trajectory->trajectory_point(i).path_point().y() -
                         trajectory->trajectory_point(i - 1).path_point().y(),
                     trajectory->trajectory_point(i).path_point().x() -
                         trajectory->trajectory_point(i - 1).path_point().x()));
    }
    point->set_relative_time(static_cast<double>(i) *
                             FLAGS_prediction_trajectory_time_resolution);
    if (i == 0) {
      point->set_v(latest_feature_ptr->speed());
    } else {
      double diff_x = point_x - prev_x;
      double diff_y = point_y - prev_y;
      point->set_v(std::hypot(diff_x, diff_y) /
                   FLAGS_prediction_trajectory_time_resolution);
    }
  }

  return true;
}

bool JointlyPredictionPlanningEvaluator::ExtractObstacleHistory(
    Obstacle* obstacle_ptr,
    std::vector<std::pair<double, double>>* pos_history) {
  pos_history->resize(20, {0.0, 0.0});
  const Feature& obs_curr_feature = obstacle_ptr->latest_feature();
  double obs_curr_heading = obs_curr_feature.velocity_heading();
  std::pair<double, double> obs_curr_pos = std::make_pair(
      obs_curr_feature.position().x(), obs_curr_feature.position().y());
  for (std::size_t i = 0; i < obstacle_ptr->history_size() && i < 20; ++i) {
    const Feature& feature = obstacle_ptr->feature(i);
    if (!feature.IsInitialized()) {
      break;
    }
    pos_history->at(i) = WorldCoordToObjCoord(
        std::make_pair(feature.position().x(), feature.position().y()),
        obs_curr_pos, obs_curr_heading);
  }
  return true;
}

bool JointlyPredictionPlanningEvaluator::ExtractADCTrajectory(
    std::vector<TrajectoryPoint>* trajectory_points,
    Obstacle* obstacle_ptr,
    std::vector<std::pair<double, double>>* adc_traj_curr_pos) {
  adc_traj_curr_pos->resize(30, {0.0, 0.0});
  const Feature& obs_curr_feature = obstacle_ptr->latest_feature();
  double obs_curr_heading = obs_curr_feature.velocity_heading();
  std::pair<double, double> obs_curr_pos = std::make_pair(
      obs_curr_feature.position().x(), obs_curr_feature.position().y());
  size_t adc_traj_points_num = trajectory_points->size();
  for (size_t i = 0; i < 30; ++i) {
    if (i > adc_traj_points_num -1) {
      adc_traj_curr_pos->at(i) =
          adc_traj_curr_pos->at(adc_traj_points_num - 1);
    } else {
      adc_traj_curr_pos->at(i) = WorldCoordToObjCoord(
          std::make_pair(trajectory_points->at(i).path_point().x(),
          trajectory_points->at(i).path_point().y()),
          obs_curr_pos, obs_curr_heading);
    }
  }
  return true;
}

void JointlyPredictionPlanningEvaluator::LoadModel() {
  if (FLAGS_use_cuda && torch::cuda::is_available()) {
    ADEBUG << "CUDA is available";
    device_ = torch::Device(torch::kCUDA);
    torch_vehicle_model_ =
        torch::jit::load(FLAGS_torch_vehicle_jointly_model_file, device_);
  } else {
    torch_vehicle_model_ =
        torch::jit::load(FLAGS_torch_vehicle_jointly_model_cpu_file, device_);
  }
  torch::set_num_threads(1);

  // Fake intput for the first frame
  torch::Tensor img_tensor = torch::zeros({1, 3, 224, 224});
  torch::Tensor obstacle_pos = torch::zeros({1, 20, 2});
  torch::Tensor obstacle_pos_step = torch::zeros({1, 20, 2});
  torch::Tensor adc_trajectory = torch::zeros({1, 30, 6});
  std::vector<torch::jit::IValue> torch_inputs;

  auto X_value = c10::ivalue::Tuple::create(
      {std::move(img_tensor.to(device_)), std::move(obstacle_pos.to(device_)),
       std::move(obstacle_pos_step.to(device_))});
  torch_inputs.push_back(c10::ivalue::Tuple::create(
      {X_value, std::move(adc_trajectory.to(device_))}));
  // Run one inference to avoid very slow first inference later
  torch_default_output_tensor_ =
      torch_vehicle_model_.forward(torch_inputs).toTensor().to(torch::kCPU);
}

}  // namespace prediction
}  // namespace apollo
