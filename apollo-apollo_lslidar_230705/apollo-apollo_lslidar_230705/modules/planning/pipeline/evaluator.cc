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

#include <limits>
#include <vector>

#include "cyber/common/file.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/trajectory_evaluator.h"
#include "modules/planning/pipeline/evaluator_logger.h"

DEFINE_double(trajectory_delta_t, 0.2,
              "delta time(sec) between trajectory points");

namespace apollo {
namespace planning {

void Evaluator::Init() {
  start_time_ = std::chrono::system_clock::now();
  std::time_t now = std::time(nullptr);
  EvaluatorLogger::GetStream()
      << "UTC date and time: " << std::asctime(std::gmtime(&now))
      << "Local date and time: " << std::asctime(std::localtime(&now));
}

void Evaluator::Close() {
  auto end_time = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed_seconds = end_time - start_time_;
  EvaluatorLogger::GetStream()
      << "Time elapsed(sec): " << elapsed_seconds.count() << std::endl
      << std::endl;
}

void Evaluator::Evaluate(const std::string& source_file) {
  EvaluatorLogger::GetStream() << "Processing: " << source_file << std::endl;

  const std::string& source_filename =
      source_file.substr(source_file.find_last_of("/") + 1);

  cyber::common::GetProtoFromFile(source_file, &learning_data_);

  for (int i = 0; i < learning_data_.learning_data_frame_size(); ++i) {
    auto learning_data_frame = learning_data_.mutable_learning_data_frame(i);
    if (learning_data_frame->adc_trajectory_point_size() <= 0) {
      continue;
    }
    const double start_point_timestamp_sec =
        learning_data_frame
            ->adc_trajectory_point(
                learning_data_frame->adc_trajectory_point_size() - 1)
            .timestamp_sec();

    // evaluate adc trajectory
    trajectory_evaluator_.EvaluateADCTrajectory(start_point_timestamp_sec,
                                                FLAGS_trajectory_delta_t,
                                                learning_data_frame);

    // evaluate adc future trajectory
    std::vector<TrajectoryPointFeature> adc_future_trajectory;
    for (const auto& tp : learning_data_.learning_data_frame(i)
                              .output()
                              .adc_future_trajectory_point()) {
      adc_future_trajectory.push_back(tp);
    }
    std::vector<TrajectoryPointFeature> evaluated_adc_future_trajectory;
    trajectory_evaluator_.EvaluateADCFutureTrajectory(
        learning_data_.learning_data_frame(i).frame_num(),
        adc_future_trajectory, start_point_timestamp_sec,
        FLAGS_trajectory_delta_t, &evaluated_adc_future_trajectory);
    learning_data_frame->mutable_output()->clear_adc_future_trajectory_point();
    for (const auto& tp : evaluated_adc_future_trajectory) {
      auto adc_future_trajectory_point =
          learning_data_frame->mutable_output()
              ->add_adc_future_trajectory_point();
      adc_future_trajectory_point->set_timestamp_sec(tp.timestamp_sec());
      adc_future_trajectory_point->mutable_trajectory_point()->CopyFrom(
          tp.trajectory_point());
    }

    // evaluate obstacle trajectory
    trajectory_evaluator_.EvaluateObstacleTrajectory(start_point_timestamp_sec,
                                                     FLAGS_trajectory_delta_t,
                                                     learning_data_frame);

    // evaluate obstacle prediction trajectory
    trajectory_evaluator_.EvaluateObstaclePredictionTrajectory(
        start_point_timestamp_sec, FLAGS_trajectory_delta_t,
        learning_data_frame);
  }

  WriteOutData(source_filename, learning_data_);
}

void Evaluator::WriteOutData(const std::string& source_filename,
                             const LearningData& learning_data) {
  const std::string file = FLAGS_planning_data_dir + "/" + source_filename;
  cyber::common::SetProtoToBinaryFile(learning_data, file);
  // cyber::common::SetProtoToASCIIFile(learning_data, file + ".txt");
  learning_data_.Clear();
}

}  // namespace planning
}  // namespace apollo
