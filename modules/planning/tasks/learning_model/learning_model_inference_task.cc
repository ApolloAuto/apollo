/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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
 * @file
 **/

#include "modules/planning/tasks/learning_model/learning_model_inference_task.h"

#include <limits>
#include <string>

#include "absl/strings/str_cat.h"

#include "modules/planning/proto/learning_data.pb.h"
#include "modules/planning/proto/planning_config.pb.h"

#include "modules/planning/learning_based/model_inference/trajectory_imitation_inference.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;

LearningModelInferenceTask::LearningModelInferenceTask(
    const TaskConfig &config) : Task(config) {
  ACHECK(config.has_learning_model_inference_task_config());
}

Status LearningModelInferenceTask::Execute(
    Frame *frame,
    ReferenceLineInfo *reference_line_info) {
  CHECK_NOTNULL(frame);

  Task::Execute(frame, reference_line_info);
  return Process(frame);
}

Status LearningModelInferenceTask::Process(Frame *frame) {
  CHECK_NOTNULL(frame);

  const auto& config = config_.learning_model_inference_task_config();

  LearningDataFrame learning_data_frame;
  learning_data_frame.CopyFrom(frame->learning_data_frame());

  ADEBUG << "LearningModelInferenceTask: frame_num["
         << learning_data_frame.frame_num()
         << "] adc_trajectory_point_size["
         << learning_data_frame.adc_trajectory_point_size() << "]";
  // for (const auto& ob : learning_data_frame.obstacle()) {
  //   AERROR << ob.DebugString();
  // }

  if (learning_data_frame.adc_trajectory_point_size() <= 0) {
    const std::string msg =
        absl::StrCat("learning_data adc_trajectory_point empty. frame_num[",
                     learning_data_frame.frame_num(), "]");
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  const double start_point_timestamp_sec =
      learning_data_frame.adc_trajectory_point(
          learning_data_frame.adc_trajectory_point_size() - 1)
      .timestamp_sec();

  ADEBUG << "start_point_timestamp_sec: " << start_point_timestamp_sec;
  // for (const auto& t : learning_data_frame.adc_trajectory_point()) {
  //   AERROR << "BEFORE: " << t.timestamp_sec();
  // }

  // evaluate adc trajectory
  trajectory_evaluator_.EvaluateADCTrajectory(
      start_point_timestamp_sec,
      config.trajectory_delta_t(),
      &learning_data_frame);

  // for (const auto& t : learning_data_frame.adc_trajectory_point()) {
  //   AERROR << "AFTER: " << t.timestamp_sec();
  // }

  // evaluate obstacle trajectory
  trajectory_evaluator_.EvaluateObstacleTrajectory(
      start_point_timestamp_sec,
      config.trajectory_delta_t(),
      &learning_data_frame);

  // evaluate obstacle prediction trajectory
  trajectory_evaluator_.EvaluateObstaclePredictionTrajectory(
      start_point_timestamp_sec,
      config.trajectory_delta_t(),
      &learning_data_frame);

  TrajectoryConvRnnInference trajectory_conv_rnn_inference(config);
  if (!trajectory_conv_rnn_inference.Inference(&learning_data_frame)) {
    const std::string msg =
        absl::StrCat("TrajectoryConvRnnInference Inference failed. frame_num[",
                     learning_data_frame.frame_num(), "]");
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  const int adc_future_trajectory_point_size =
      learning_data_frame.output().adc_future_trajectory_point_size();
  ADEBUG << "   adc_future_trajectory_point_size["
         << adc_future_trajectory_point_size << "]";
  if (adc_future_trajectory_point_size < 10) {
    const std::string msg =
        absl::StrCat("too short adc_future_trajectory_point. frame_num[",
                     learning_data_frame.frame_num(),
                     "] adc_future_trajectory_point_size[",
                     adc_future_trajectory_point_size, "]");
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  constexpr double kADCFutureTrajectoryDeltaTime = 0.02;
  std::vector<TrajectoryPointFeature> evaluated_trajectory;
  trajectory_evaluator_.EvaluateADCFutureTrajectory(
      learning_data_frame,
      start_point_timestamp_sec,
      kADCFutureTrajectoryDeltaTime,
      &evaluated_trajectory);

  std::vector<common::TrajectoryPoint> adc_future_trajectory;
  ConvertADCFutureTrajectory(evaluated_trajectory,
                             &adc_future_trajectory);
  // for (const auto& t : adc_future_trajectory) {
  //   AERROR << "FUTURE: " << t.relative_time();
  // }

  frame->set_learning_data_adc_future_trajectory_points(adc_future_trajectory);

  return Status::OK();
}

void LearningModelInferenceTask::ConvertADCFutureTrajectory(
    const std::vector<TrajectoryPointFeature>& trajectory,
    std::vector<common::TrajectoryPoint>* adc_future_trajectory) {
  adc_future_trajectory->clear();

  for (const auto& trajectory_point_feature : trajectory) {
    const auto& tp = trajectory_point_feature.trajectory_point();
    // TrajectoryPointFeature => common::TrajectoryPoint
    apollo::common::TrajectoryPoint trajectory_point;
    auto path_point = trajectory_point.mutable_path_point();
    path_point->set_x(tp.path_point().x());
    path_point->set_y(tp.path_point().y());
    path_point->set_z(tp.path_point().z());
    path_point->set_theta(tp.path_point().theta());
    // path_point->set_kappa();
    path_point->set_s(tp.path_point().s());
    // path_point->set_dkappa();
    // path_point->set_ddkappa();
    path_point->set_lane_id(tp.path_point().lane_id());
    // path_point->set_x_derivative();
    // path_point->set_y_derivative();

    trajectory_point.set_v(tp.v());
    trajectory_point.set_a(tp.a());
    trajectory_point.set_relative_time(tp.relative_time());
    // trajectory_point.set_da();
    // trajectory_point.set_steer();
    trajectory_point.mutable_gaussian_info()->CopyFrom(tp.gaussian_info());

    adc_future_trajectory->push_back(trajectory_point);
  }
}

}  // namespace planning
}  // namespace apollo
