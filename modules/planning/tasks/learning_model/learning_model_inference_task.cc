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
  //  AERROR << ob.DebugString();
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
  //   ADEBUG << "BEFORE: " << t.timestamp_sec();
  // }

  // evaluate adc trajectory
  EvaluateADCTrajectory(start_point_timestamp_sec,
                        &learning_data_frame);

  // for (const auto& t : learning_data_frame.adc_trajectory_point()) {
  //   ADEBUG << "AFTER: " << t.timestamp_sec();
  // }

  // evaluate obstacle trajectory
  EvaluateObstacleTrajectory(start_point_timestamp_sec,
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

  std::vector<common::TrajectoryPoint> adc_future_trajectory;
  GenerateADCFutureTrajectory(learning_data_frame.output(),
                              start_point_timestamp_sec,
                              &adc_future_trajectory);
  // for (const auto& t : adc_future_trajectory) {
  //   ADEBUG << "FUTURE: " << t.relative_time();
  // }

  frame->set_learning_data_adc_future_trajectory_points(adc_future_trajectory);

  return Status::OK();
}

void LearningModelInferenceTask::GenerateADCFutureTrajectory(
    const LearningOutput& learning_output,
    const double start_point_timestamp_sec,
    std::vector<common::TrajectoryPoint>* adc_future_trajectory) {
  adc_future_trajectory->clear();

  std::vector<std::pair<double, CommonTrajectoryPointFeature>> trajectory;
  for (const auto& adc_future_trajectory_point:
      learning_output.adc_future_trajectory_point()) {
    trajectory.push_back(std::make_pair(
        adc_future_trajectory_point.timestamp_sec(),
        adc_future_trajectory_point.trajectory_point()));
  }

  constexpr double kADCFutureTrajectoryDeltaT = 0.05;
  std::vector<TrajectoryPointFeature> evaluated_trajectory;
  EvaluateTrajectoryByTime(trajectory,
                           start_point_timestamp_sec,
                           kADCFutureTrajectoryDeltaT,
                           &evaluated_trajectory);

  ADEBUG << "orig adc_future_trajectory_point size["
         << trajectory.size()
         << "] evaluated size[" << evaluated_trajectory.size() << "]";

  for (const auto& evaluated_trajectory_point : evaluated_trajectory) {
    const auto& tp = evaluated_trajectory_point.trajectory_point();
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

void LearningModelInferenceTask::EvaluateTrajectoryByTime(
    const std::vector<std::pair<double, CommonTrajectoryPointFeature>>&
        trajectory,
    const double start_point_timestamp_sec,
    const double delta_time,
    std::vector<TrajectoryPointFeature>* evaluated_trajectory) {
  if (trajectory.empty() ||
      fabs(trajectory.front().first - trajectory.back().first) < delta_time) {
    return;
  }

  std::vector<apollo::common::TrajectoryPoint> updated_trajectory;
  for (const auto& tp : trajectory) {
    // CommonTrajectoryPointFeature => common::TrajectoryPoint
    apollo::common::TrajectoryPoint trajectory_point;
    auto path_point = trajectory_point.mutable_path_point();
    path_point->set_x(tp.second.path_point().x());
    path_point->set_y(tp.second.path_point().y());
    path_point->set_z(tp.second.path_point().z());
    path_point->set_theta(tp.second.path_point().theta());
    path_point->set_s(tp.second.path_point().s());
    path_point->set_lane_id(tp.second.path_point().lane_id());
    trajectory_point.set_v(tp.second.v());
    trajectory_point.set_a(tp.second.a());
    double relative_time = tp.first - start_point_timestamp_sec;
    trajectory_point.set_relative_time(relative_time);
    trajectory_point.mutable_gaussian_info()->CopyFrom(
        tp.second.gaussian_info());
    updated_trajectory.push_back(trajectory_point);
  }

  if (trajectory.front().first > trajectory.back().first) {
    std::reverse(updated_trajectory.begin(), updated_trajectory.end());
  }

  DiscretizedTrajectory discretized_trajectory;
  double last_relative_time = std::numeric_limits<double>::lowest();
  for (const auto& tp : updated_trajectory) {
    // filter out  abnormal perception data
    if (tp.relative_time() > last_relative_time) {
      discretized_trajectory.AppendTrajectoryPoint(tp);
    }
    last_relative_time = tp.relative_time();
  }

  const int low_bound =
      ceil(updated_trajectory.front().relative_time() / delta_time);
  const int high_bound =
      floor(updated_trajectory.back().relative_time() / delta_time);
  ADEBUG << "low[" << low_bound << "] high[" << high_bound << "]";
  for (int i = low_bound; i <= high_bound; ++i) {
    double timestamp_sec = start_point_timestamp_sec + i * delta_time;
    double relative_time = i * delta_time;
    auto tp = discretized_trajectory.Evaluate(relative_time);

    // common::TrajectoryPoint => TrajectoryPointFeature
    TrajectoryPointFeature trajectory_point;
    trajectory_point.set_timestamp_sec(timestamp_sec);
    auto path_point =
        trajectory_point.mutable_trajectory_point()->mutable_path_point();
    path_point->set_x(tp.path_point().x());
    path_point->set_y(tp.path_point().y());
    path_point->set_z(tp.path_point().z());
    path_point->set_theta(tp.path_point().theta());
    path_point->set_s(tp.path_point().s());
    path_point->set_lane_id(tp.path_point().lane_id());
    trajectory_point.mutable_trajectory_point()->set_v(tp.v());
    trajectory_point.mutable_trajectory_point()->set_a(tp.a());
    trajectory_point.mutable_trajectory_point()->set_relative_time(
        tp.relative_time());
    trajectory_point.mutable_trajectory_point()->mutable_gaussian_info()
                                               ->CopyFrom(tp.gaussian_info());

    evaluated_trajectory->push_back(trajectory_point);
  }
}

void LearningModelInferenceTask::EvaluateADCTrajectory(
    const double start_point_timestamp_sec,
    LearningDataFrame* learning_data_frame) {
  std::vector<std::pair<double, CommonTrajectoryPointFeature>> trajectory;
  for (const auto& adc_tp : learning_data_frame->adc_trajectory_point()) {
    trajectory.push_back(std::make_pair(adc_tp.timestamp_sec(),
                                        adc_tp.trajectory_point()));
  }

  if (trajectory.size() <= 1) {
    const std::string msg =
        absl::StrCat("too short adc_trajectory_point. frame_num[",
                     learning_data_frame->frame_num(),
                     "] size[", trajectory.size(), "]");
    AERROR << msg;
    return;
  }

  learning_data_frame->clear_adc_trajectory_point();

  const auto& config = config_.learning_model_inference_task_config();
  if (fabs(trajectory.front().first - start_point_timestamp_sec) <=
      config.trajectory_delta_t()) {
    auto adc_trajectory_point = learning_data_frame->add_adc_trajectory_point();
    adc_trajectory_point->set_timestamp_sec(trajectory.back().first);
    adc_trajectory_point->mutable_trajectory_point()->CopyFrom(
        trajectory.back().second);

    const std::string msg =
        absl::StrCat("too short adc_trajectory. frame_num[",
                     learning_data_frame->frame_num(), "] size[",
                     trajectory.size(), "] timestamp_diff[",
                     start_point_timestamp_sec - trajectory.front().first,
                     "]");
    AERROR << msg;
    return;
  }

  std::vector<TrajectoryPointFeature> evaluated_trajectory;
  EvaluateTrajectoryByTime(trajectory,
                           start_point_timestamp_sec,
                           config.trajectory_delta_t(),
                           &evaluated_trajectory);
  ADEBUG << "frame_num[" << learning_data_frame->frame_num()
         << "] orig adc_trajectory_point size[" << trajectory.size()
         << "] evaluated size[" << evaluated_trajectory.size() << "]";

  for (const auto& tp : evaluated_trajectory) {
    if (tp.trajectory_point().relative_time() <= 0.0 &&
        tp.trajectory_point().relative_time() >=
            -FLAGS_trajectory_time_length) {
      auto adc_trajectory_point =
          learning_data_frame->add_adc_trajectory_point();
      adc_trajectory_point->set_timestamp_sec(tp.timestamp_sec());
      adc_trajectory_point->mutable_trajectory_point()
                          ->CopyFrom(tp.trajectory_point());
    } else {
      const std::string msg =
          absl::StrCat("DISCARD adc_trajectory_point. frame_num[",
                       learning_data_frame->frame_num(), "] size[",
                       evaluated_trajectory.size(), "] relative_time[",
                       tp.trajectory_point().relative_time() , "]");
      AERROR << msg;
    }
  }
}

void LearningModelInferenceTask::EvaluateObstacleTrajectory(
    const double start_point_timestamp_sec,
    LearningDataFrame* learning_data_frame) {
  for (int i = 0; i < learning_data_frame->obstacle_size(); ++i) {
   const int obstacle_id = learning_data_frame->obstacle(i).id();
   const auto obstacle_trajectory =
        learning_data_frame->obstacle(i).obstacle_trajectory();

    std::vector<std::pair<double, CommonTrajectoryPointFeature>> trajectory;
    for (const auto& perception_obstacle :
        obstacle_trajectory.perception_obstacle_history()) {
      CommonTrajectoryPointFeature trajectory_point;
      trajectory_point.mutable_path_point()->set_x(
          perception_obstacle.position().x());
      trajectory_point.mutable_path_point()->set_y(
          perception_obstacle.position().y());
      trajectory_point.mutable_path_point()->set_z(
          perception_obstacle.position().z());
      trajectory_point.mutable_path_point()->set_theta(
          perception_obstacle.theta());

      const double v = std::sqrt(
          perception_obstacle.velocity().x() *
          perception_obstacle.velocity().x() +
          perception_obstacle.velocity().y() *
          perception_obstacle.velocity().y());
      trajectory_point.set_v(v);

      const double a = std::sqrt(
          perception_obstacle.acceleration().x() *
          perception_obstacle.acceleration().x() +
          perception_obstacle.acceleration().y() *
          perception_obstacle.acceleration().y());
      trajectory_point.set_a(a);

      trajectory.push_back(std::make_pair(perception_obstacle.timestamp_sec(),
                                          trajectory_point));
    }

    const auto& config = config_.learning_model_inference_task_config();

    if (fabs(trajectory.front().first - start_point_timestamp_sec) <=
        config.trajectory_delta_t()) {
      ADEBUG << "too short obstacle_trajectory. frame_num["
             << learning_data_frame->frame_num()
             << "] obstacle_id[" << obstacle_id << "] size["
             << trajectory.size() << "] timestamp_diff["
             << start_point_timestamp_sec - trajectory.front().first << "]";
      continue;
    }

    std::vector<TrajectoryPointFeature> evaluated_trajectory;
    EvaluateTrajectoryByTime(trajectory,
                             start_point_timestamp_sec,
                             config.trajectory_delta_t(),
                             &evaluated_trajectory);

    ADEBUG << "frame[" << learning_data_frame.frame_num()
           << "] orig obstacle_trajectory[" << trajectory.size()
           << "] evaluated size[" << evaluated_trajectory.size() << "]";

    // update learning_data
    learning_data_frame->mutable_obstacle(i)
                       ->mutable_obstacle_trajectory()
                       ->clear_evaluated_trajectory_point();
    for (const auto& tp : evaluated_trajectory) {
      auto evaluated_trajectory_point =
          learning_data_frame->mutable_obstacle(i)
                             ->mutable_obstacle_trajectory()
                             ->add_evaluated_trajectory_point();
      evaluated_trajectory_point->set_timestamp_sec(tp.timestamp_sec());
      evaluated_trajectory_point->mutable_trajectory_point()
                                ->CopyFrom(tp.trajectory_point());
    }
  }
}

}  // namespace planning
}  // namespace apollo
