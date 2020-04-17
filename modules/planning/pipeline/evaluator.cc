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
    const std::vector<std::pair<double, TrajectoryPoint>>& trajectory,
    const double delta_time,
    std::vector<std::pair<double, TrajectoryPoint>>* evaluated_trajectory) {
  if (trajectory.empty() ||
      fabs(trajectory.front().first - trajectory.back().first) < 1.0) {
    return;
  }

  double start_point_timestamp_sec = trajectory.front().first;
  int direction =
      (trajectory.front().first < trajectory.back().first) ? 1 : -1;

  std::vector<TrajectoryPoint> updated_trajectory;
  for (const auto& tp : trajectory) {
    TrajectoryPoint trajectory_point;
    trajectory_point.CopyFrom(tp.second);
    double relative_time = tp.first - start_point_timestamp_sec;
    trajectory_point.set_relative_time(relative_time);
    updated_trajectory.push_back(trajectory_point);
  }
  if (direction < 0) {
    std::reverse(updated_trajectory.begin(), updated_trajectory.end());
  }

  DiscretizedTrajectory discretized_trajectory;
  for (const auto& tp : updated_trajectory) {
    discretized_trajectory.AppendTrajectoryPoint(tp);
  }

  double timestamp_sec = start_point_timestamp_sec;
  double relative_time = 0.0;
  while ((direction > 0 && timestamp_sec < trajectory.back().first) ||
      (direction < 0 && timestamp_sec > trajectory.back().first)) {
    auto trajectory_point = discretized_trajectory.Evaluate(relative_time);
    evaluated_trajectory->push_back(
        std::make_pair(timestamp_sec, trajectory_point));
    timestamp_sec += direction * delta_time;
    relative_time += direction * delta_time;
  }
}

void Evaluator::EvaluateADCTrajectory(
    LearningDataFrame* learning_data_frame) {
  std::vector<std::pair<double, TrajectoryPoint>> trajectory;
  for (int i = 0; i < learning_data_frame->adc_trajectory_point_size(); i++) {
    ADCTrajectoryPoint adc_tp =
        learning_data_frame->adc_trajectory_point(i);
    trajectory.push_back(std::make_pair(adc_tp.timestamp_sec(),
                                        adc_tp.trajectory_point()));
  }

  std::vector<std::pair<double, TrajectoryPoint>> evaluated_trajectory;
  EvaluateTrajectoryByTime(trajectory,
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
  std::vector<std::pair<double, TrajectoryPoint>> trajectory;
  for (int i = 0; i <
      learning_data_frame->output().adc_future_trajectory_point_size(); i++) {
    ADCTrajectoryPoint adc_tp =
        learning_data_frame->output().adc_future_trajectory_point(i);
    trajectory.push_back(std::make_pair(adc_tp.timestamp_sec(),
                                        adc_tp.trajectory_point()));
  }

  std::vector<std::pair<double, TrajectoryPoint>> evaluated_trajectory;
  EvaluateTrajectoryByTime(trajectory,
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

}  // namespace planning
}  // namespace apollo
