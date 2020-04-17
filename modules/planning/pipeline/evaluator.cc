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

#include "modules/planning/pipeline/evaluator.h"

#include <string>

#include "cyber/common/file.h"
#include "modules/planning/common/trajectory/discretized_trajectory.h"

DEFINE_string(planning_data_dir, "/apollo/modules/planning/data/",
              "Prefix of files to store learning_data_frame data");
DEFINE_double(trajectory_delta_t, 0.2,
             "delta time(sec) between trajectory points");
DEFINE_bool(enable_obstacle_trajectory, true,
            "enable obstacle_trajectory evaluation by time");

namespace apollo {
namespace planning {

using apollo::common::TrajectoryPoint;

void Evaluator::Init() {
}

void Evaluator::Evaluate(const std::string& source_file) {
  source_filename_ =
      source_file.substr(source_file.find_last_of("/") + 1);
  cyber::common::GetProtoFromFile(source_file,
                                  &learning_data_);
  for (int i = 0; i < learning_data_.learning_data_size(); ++i) {
    auto learning_data_frame = learning_data_.mutable_learning_data(i);

    // evaluate adc trajectory
    EvaluateADCTrajectory(learning_data_frame);

    // evaluate adc future trajectory
    EvaluateADCFutureTrajectory(learning_data_frame);

    // evaluate obstacle trajectory
    if (learning_data_frame->adc_trajectory_point_size() > 0) {
      double start_point_timestamp_sec =
          learning_data_frame->adc_trajectory_point(0).timestamp_sec();
      EvaluateObstacleTrajectory(start_point_timestamp_sec,
                                 learning_data_frame);
    }
  }
}

void Evaluator::WriteOutLearningData(
    const LearningData& learning_data) {
  const std::string file_name =
      FLAGS_planning_data_dir + source_filename_;
  cyber::common::SetProtoToBinaryFile(learning_data, file_name);
  cyber::common::SetProtoToASCIIFile(learning_data, file_name + ".txt");
  learning_data_.Clear();
}

void Evaluator::Close() {
  WriteOutLearningData(learning_data_);
}

void Evaluator::EvaluateTrajectoryByTime(
    const std::vector<std::pair<double, TrajectoryPointFeature>>& trajectory,
    const double start_point_timestamp_sec,
    const double delta_time,
    std::vector<std::pair<double, TrajectoryPointFeature>>*
        evaluated_trajectory) {
  if (trajectory.empty() ||
      fabs(trajectory.front().first - trajectory.back().first) < 1.0) {
    return;
  }

  int direction =
      (trajectory.front().first < trajectory.back().first) ? 1 : -1;

  std::vector<TrajectoryPoint> updated_trajectory;
  for (const auto& tp : trajectory) {
    // TrajectoryPointFeature => common::Trajectory
    TrajectoryPoint trajectory_point;
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

  DiscretizedTrajectory discretized_trajectory;
  if (direction < 0) {
    std::reverse(updated_trajectory.begin(), updated_trajectory.end());
  }
  for (const auto& tp : updated_trajectory) {
    discretized_trajectory.AppendTrajectoryPoint(tp);
  }

  double timestamp_sec = start_point_timestamp_sec;
  double relative_time = 0.0;
  while ((direction > 0 && timestamp_sec < trajectory.back().first) ||
      (direction < 0 && timestamp_sec > trajectory.back().first)) {
    auto tp = discretized_trajectory.Evaluate(relative_time);

    // common::TrajectoryPoint => TrajectoryPointFeature
    TrajectoryPointFeature trajectory_point;
    trajectory_point.mutable_path_point()->set_x(tp.path_point().x());
    trajectory_point.mutable_path_point()->set_y(tp.path_point().y());
    trajectory_point.mutable_path_point()->set_z(tp.path_point().z());
    trajectory_point.mutable_path_point()->set_theta(tp.path_point().theta());
    trajectory_point.mutable_path_point()->set_s(tp.path_point().s());
    trajectory_point.mutable_path_point()->set_lane_id(
        tp.path_point().lane_id());
    trajectory_point.set_v(tp.v());
    trajectory_point.set_a(tp.a());
    trajectory_point.set_relative_time(tp.relative_time());
    trajectory_point.mutable_gaussian_info()->CopyFrom(tp.gaussian_info());

    evaluated_trajectory->push_back(
        std::make_pair(timestamp_sec, trajectory_point));
    timestamp_sec += direction * delta_time;
    relative_time += direction * delta_time;
  }
}

void Evaluator::EvaluateADCTrajectory(
    LearningDataFrame* learning_data_frame) {
  std::vector<std::pair<double, TrajectoryPointFeature>> trajectory;
  for (int i = 0; i < learning_data_frame->adc_trajectory_point_size(); i++) {
    ADCTrajectoryPoint adc_tp =
        learning_data_frame->adc_trajectory_point(i);
    trajectory.push_back(std::make_pair(adc_tp.timestamp_sec(),
                                        adc_tp.trajectory_point()));
  }
  if (trajectory.size() < 3 ||
      fabs(trajectory.front().first - trajectory.back().first) <=
          FLAGS_trajectory_delta_t) {
    return;
  }

  std::vector<std::pair<double, TrajectoryPointFeature>> evaluated_trajectory;
  EvaluateTrajectoryByTime(trajectory,
                           trajectory.front().first,
                           FLAGS_trajectory_delta_t,
                           &evaluated_trajectory);
  ADEBUG << "orig adc_trajectory["
         << learning_data_frame->adc_trajectory_point_size()
         << "] evaluated[" << evaluated_trajectory.size() << "]";

  // update learning_data
  learning_data_frame->clear_adc_trajectory_point();
  for (const auto& tp : evaluated_trajectory) {
    auto adc_trajectory_point = learning_data_frame->add_adc_trajectory_point();
    adc_trajectory_point->set_timestamp_sec(tp.first);
    adc_trajectory_point->mutable_trajectory_point()->CopyFrom(tp.second);
  }
}

void Evaluator::EvaluateADCFutureTrajectory(
    LearningDataFrame* learning_data_frame) {
  std::vector<std::pair<double, TrajectoryPointFeature>> trajectory;
  for (int i = 0; i <
      learning_data_frame->output().adc_future_trajectory_point_size(); i++) {
    ADCTrajectoryPoint adc_tp =
        learning_data_frame->output().adc_future_trajectory_point(i);
    trajectory.push_back(std::make_pair(adc_tp.timestamp_sec(),
                                        adc_tp.trajectory_point()));
  }
  if (trajectory.size() < 3 ||
      fabs(trajectory.front().first - trajectory.back().first) <=
          FLAGS_trajectory_delta_t) {
    return;
  }

  std::vector<std::pair<double, TrajectoryPointFeature>> evaluated_trajectory;
  EvaluateTrajectoryByTime(trajectory,
                           trajectory.front().first,
                           FLAGS_trajectory_delta_t,
                           &evaluated_trajectory);

  ADEBUG << "orig adc_future_trajectory["
        << learning_data_frame->output().adc_future_trajectory_point_size()
        << "] evaluated[" << evaluated_trajectory.size() << "]";

  // update learning_data
  learning_data_frame->mutable_output()->clear_adc_future_trajectory_point();
  for (const auto& tp : evaluated_trajectory) {
    auto adc_future_trajectory_point =
        learning_data_frame->mutable_output()
                           ->add_adc_future_trajectory_point();
    adc_future_trajectory_point->set_timestamp_sec(tp.first);
    adc_future_trajectory_point->mutable_trajectory_point()
                               ->CopyFrom(tp.second);
  }
}

void Evaluator::EvaluateObstacleTrajectory(
    const double start_point_timestamp_sec,
    LearningDataFrame* learning_data_frame) {
  if (!FLAGS_enable_obstacle_trajectory) {
    return;
  }

  for (int i = 0; i < learning_data_frame->obstacle_size(); ++i) {
    const auto obstacle_trajectory =
        learning_data_frame->obstacle(i).obstacle_trajectory();

    std::vector<std::pair<double, TrajectoryPointFeature>> trajectory;
    for (int j = 0; j <
        obstacle_trajectory.perception_obstacle_history_size(); ++j) {
      const auto perception_obstacle =
          obstacle_trajectory.perception_obstacle_history(j);

      TrajectoryPointFeature trajectory_point;
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
    if (trajectory.size() < 3 ||
        fabs(trajectory.front().first - trajectory.back().first) <=
            FLAGS_trajectory_delta_t) {
      continue;
    }

    std::vector<std::pair<double, TrajectoryPointFeature>> evaluated_trajectory;
    EvaluateTrajectoryByTime(trajectory,
                             start_point_timestamp_sec,
                             FLAGS_trajectory_delta_t,
                             &evaluated_trajectory);

    ADEBUG << "orig obstacle_trajectory["
           << obstacle_trajectory.perception_obstacle_history_size()
           << "] evaluated[" << evaluated_trajectory.size() << "]";

    // update learning_data
    learning_data_frame->mutable_obstacle(i)
                       ->mutable_obstacle_trajectory()
                       ->clear_evaluated_trajectory_point();
    for (const auto& tp : evaluated_trajectory) {
      auto evaluated_trajectory_point =
          learning_data_frame->mutable_obstacle(i)
                             ->mutable_obstacle_trajectory()
                             ->add_evaluated_trajectory_point();
      evaluated_trajectory_point->CopyFrom(tp.second);
    }
  }
}

}  // namespace planning
}  // namespace apollo
