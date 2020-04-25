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

#include <sstream>

#include "cyber/common/file.h"
#include "modules/planning/common/trajectory/discretized_trajectory.h"

DEFINE_string(planning_data_dir, "/apollo/modules/planning/data/",
              "Prefix of files to store learning_data_frame data");
DEFINE_double(trajectory_delta_t, 0.2,
             "delta time(sec) between trajectory points");
DEFINE_bool(enable_evaluate_obstacle_trajectory, true,
            "enable obstacle_trajectory evaluation by time");

namespace apollo {
namespace planning {

void Evaluator::Init() {
  log_file_.open(FLAGS_planning_data_dir + "/output_data_evaluated.log",
                 std::ios_base::out | std::ios_base::app);
  start_time_ = std::chrono::system_clock::now();
  std::time_t now = std::time(nullptr);
  log_file_ << "UTC date and time: " << std::asctime(std::gmtime(&now))
            << "Local date and time: "
            << std::asctime(std::localtime(&now));
}

void Evaluator::Close() {
  auto end_time = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed_seconds = end_time - start_time_;
  log_file_ << "Time elapsed(sec): " << elapsed_seconds.count()
            << std::endl << std::endl;
  log_file_.close();
}

void Evaluator::Evaluate(const std::string& source_file) {
  log_file_ << "Processing: " << source_file << std::endl;

  const std::string& source_filename =
      source_file.substr(source_file.find_last_of("/") + 1);

  cyber::common::GetProtoFromFile(source_file,
                                  &learning_data_);

  for (int i = 0; i < learning_data_.learning_data_size(); ++i) {
    auto learning_data_frame = learning_data_.mutable_learning_data(i);
    if (learning_data_frame->adc_trajectory_point_size() <= 0) {
      continue;
    }
    const double start_point_timestamp_sec =
        learning_data_frame
            ->adc_trajectory_point(
                learning_data_frame->adc_trajectory_point_size()-1)
            .timestamp_sec();

    // evaluate adc trajectory
    EvaluateADCTrajectory(start_point_timestamp_sec,
                          learning_data_frame);

    // evaluate adc future trajectory
    EvaluateADCFutureTrajectory(start_point_timestamp_sec,
                                learning_data_frame);

    // evaluate obstacle trajectory
    EvaluateObstacleTrajectory(start_point_timestamp_sec,
                               learning_data_frame);

    // evaluate obstacle prediction trajectory
    EvaluateObstaclePredictionTrajectory(start_point_timestamp_sec,
                                         learning_data_frame);
  }

  WriteOutData(source_filename, learning_data_);
}

void Evaluator::WriteOutData(
    const std::string& source_filename,
    const LearningData& learning_data) {
  const std::string file =
      FLAGS_planning_data_dir + "/" + source_filename;
  cyber::common::SetProtoToBinaryFile(learning_data, file);
  // cyber::common::SetProtoToASCIIFile(learning_data, file + ".txt");
  learning_data_.Clear();
}

void Evaluator::EvaluateTrajectoryByTime(
    const int frame_num,
    const std::vector<std::pair<double, TrajectoryPointFeature>>& trajectory,
    const double start_point_timestamp_sec,
    const double delta_time,
    std::vector<TrajectoryPoint>* evaluated_trajectory) {
  if (trajectory.empty() ||
      fabs(trajectory.front().first - trajectory.back().first) <
          FLAGS_trajectory_delta_t) {
    return;
  }

  std::vector<apollo::common::TrajectoryPoint> updated_trajectory;
  for (const auto& tp : trajectory) {
    // TrajectoryPointFeature => common::Trajectory
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
  for (const auto& tp : updated_trajectory) {
    discretized_trajectory.AppendTrajectoryPoint(tp);
  }

  const int low_bound =
      ceil(updated_trajectory.front().relative_time() / delta_time);
  const int high_bound =
      floor(updated_trajectory.back().relative_time() / delta_time);
  ADEBUG << "frame_num[" << frame_num
         << "] low[" << low_bound << "] high[" << high_bound << "]";
  for (int i = low_bound; i <= high_bound; ++i) {
    double timestamp_sec = start_point_timestamp_sec + i * delta_time;
    double relative_time = i * delta_time;
    auto tp = discretized_trajectory.Evaluate(relative_time);

    // common::TrajectoryPoint => TrajectoryPoint
    TrajectoryPoint trajectory_point;
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

void Evaluator::EvaluateADCTrajectory(
    const double start_point_timestamp_sec,
    LearningDataFrame* learning_data_frame) {
  std::vector<std::pair<double, TrajectoryPointFeature>> trajectory;
  for (int i = 0; i < learning_data_frame->adc_trajectory_point_size(); i++) {
    ADCTrajectoryPoint adc_tp =
        learning_data_frame->adc_trajectory_point(i);
    trajectory.push_back(std::make_pair(adc_tp.timestamp_sec(),
                                        adc_tp.trajectory_point()));
  }

  if (trajectory.size() <= 1) {
    std::ostringstream msg;
    msg << "too few adc_trajectory_point. frame_num{"
        << learning_data_frame->frame_num() << "] size["
        << trajectory.size() << "]";
    AERROR << msg.str();
    log_file_ << msg.str() << std::endl;
    return;
  }

  learning_data_frame->clear_adc_trajectory_point();
  if (fabs(trajectory.front().first - start_point_timestamp_sec) <=
      FLAGS_trajectory_delta_t) {
    auto adc_trajectory_point = learning_data_frame->add_adc_trajectory_point();
    adc_trajectory_point->set_timestamp_sec(trajectory.back().first);
    adc_trajectory_point->mutable_trajectory_point()->CopyFrom(
        trajectory.back().second);

    std::ostringstream msg;
    msg << "too short adc_trajectory. frame_num["
        << learning_data_frame->frame_num() << "] size["
        << trajectory.size() << "] timestamp_diff["
        << start_point_timestamp_sec - trajectory.front().first << "]";
    AERROR << msg.str();
    log_file_ << msg.str() << std::endl;
    return;
  }

  std::vector<TrajectoryPoint> evaluated_trajectory;
  EvaluateTrajectoryByTime(learning_data_frame->frame_num(),
                           trajectory,
                           start_point_timestamp_sec,
                           FLAGS_trajectory_delta_t,
                           &evaluated_trajectory);
  ADEBUG << "frame_num[" << learning_data_frame->frame_num()
         << "] orig adc_trajectory[" << trajectory.size()
         << "] evaluated[" << evaluated_trajectory.size() << "]";

  for (const auto& tp : evaluated_trajectory) {
    auto adc_trajectory_point = learning_data_frame->add_adc_trajectory_point();
    adc_trajectory_point->set_timestamp_sec(tp.timestamp_sec());
    adc_trajectory_point->mutable_trajectory_point()
                        ->CopyFrom(tp.trajectory_point());
  }
}

void Evaluator::EvaluateADCFutureTrajectory(
    const double start_point_timestamp_sec,
    LearningDataFrame* learning_data_frame) {
  const int size =
      learning_data_frame->output().adc_future_trajectory_point_size();
  if (size < 60) {
    std::ostringstream msg;
    msg << "less adc_future_trajectory_points. frame_num["
        << learning_data_frame->frame_num() << "] size[" << size << "]";
    AERROR << msg.str();
    log_file_ << msg.str() << std::endl;
  }
  std::vector<std::pair<double, TrajectoryPointFeature>> trajectory;
  for (int i = 0; i <
      learning_data_frame->output().adc_future_trajectory_point_size(); i++) {
    ADCTrajectoryPoint adc_tp =
        learning_data_frame->output().adc_future_trajectory_point(i);
    trajectory.push_back(std::make_pair(adc_tp.timestamp_sec(),
                                        adc_tp.trajectory_point()));
  }

  learning_data_frame->mutable_output()->clear_adc_future_trajectory_point();
  if (trajectory.size() <= 1) {
    std::ostringstream msg;
    msg << "too short adc_future_trajectory. frame_num["
        << learning_data_frame->frame_num() << "] size["
        << trajectory.size() << "]";
    AERROR << msg.str();
    log_file_ << msg.str() << std::endl;
    return;
  }
  if (fabs(trajectory.back().first - start_point_timestamp_sec) <=
      FLAGS_trajectory_delta_t) {
    std::ostringstream msg;
    msg << "too short adc_future_trajectory. frame_num["
        << learning_data_frame->frame_num() << "] size["
        << trajectory.size() << "] timestamp_diff["
        << trajectory.back().first - start_point_timestamp_sec << "]";
    AERROR << msg.str();
    log_file_ << msg.str() << std::endl;
    return;
  }

  std::vector<TrajectoryPoint> evaluated_trajectory;
  EvaluateTrajectoryByTime(learning_data_frame->frame_num(),
                           trajectory,
                           start_point_timestamp_sec,
                           FLAGS_trajectory_delta_t,
                           &evaluated_trajectory);

  ADEBUG << "frame_num[" << learning_data_frame->frame_num()
         << "] orig adc_future_trajectory[" << trajectory.size()
         << "] evaluated[" << evaluated_trajectory.size() << "]";

  for (const auto& tp : evaluated_trajectory) {
    auto adc_future_trajectory_point =
        learning_data_frame->mutable_output()
                           ->add_adc_future_trajectory_point();
    adc_future_trajectory_point->set_timestamp_sec(tp.timestamp_sec());
    adc_future_trajectory_point->mutable_trajectory_point()
                               ->CopyFrom(tp.trajectory_point());
  }
}

void Evaluator::EvaluateObstacleTrajectory(
    const double start_point_timestamp_sec,
    LearningDataFrame* learning_data_frame) {
  if (!FLAGS_enable_evaluate_obstacle_trajectory) {
    return;
  }

  for (int i = 0; i < learning_data_frame->obstacle_size(); ++i) {
    const int obstacle_id = learning_data_frame->obstacle(i).id();
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
    if (fabs(trajectory.front().first - start_point_timestamp_sec) <=
        FLAGS_trajectory_delta_t) {
      ADEBUG << "too short obstacle_trajectory. frame_num["
             << learning_data_frame->frame_num()
             << "] obstacle_id[" << obstacle_id << "] size["
             << trajectory.size() << "] timestamp_diff["
             << start_point_timestamp_sec - trajectory.front().first << "]";
      continue;
    }

    std::vector<TrajectoryPoint> evaluated_trajectory;
    EvaluateTrajectoryByTime(learning_data_frame->frame_num(),
                             trajectory,
                             start_point_timestamp_sec,
                             FLAGS_trajectory_delta_t,
                             &evaluated_trajectory);

    ADEBUG << "frame_num[" << learning_data_frame->frame_num()
           << "] obstacle_id[" << obstacle_id
           << "] orig obstacle_trajectory[" << trajectory.size()
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
      evaluated_trajectory_point->set_timestamp_sec(tp.timestamp_sec());
      evaluated_trajectory_point->mutable_trajectory_point()
                                ->CopyFrom(tp.trajectory_point());
    }
  }
}

void Evaluator::EvaluateObstaclePredictionTrajectory(
    const double start_point_timestamp_sec,
    LearningDataFrame* learning_data_frame) {
  if (!FLAGS_enable_evaluate_obstacle_trajectory) {
    return;
  }

  for (int i = 0; i < learning_data_frame->obstacle_size(); ++i) {
    const int obstacle_id = learning_data_frame->obstacle(i).id();
    const auto obstacle_prediction =
         learning_data_frame->obstacle(i).obstacle_prediction();
    for (int j = 0; j < obstacle_prediction.trajectory_size(); ++j) {
      const auto obstacle_prediction_trajectory =
          obstacle_prediction.trajectory(j);

      std::vector<std::pair<double, TrajectoryPointFeature>> trajectory;
      for (int k = 0; k <
          obstacle_prediction_trajectory.trajectory_point_size(); ++k) {
        const double timestamp_sec = obstacle_prediction.timestamp_sec() +
            obstacle_prediction_trajectory.trajectory_point(k)
                                          .trajectory_point()
                                          .relative_time();
        trajectory.push_back(std::make_pair(
            timestamp_sec,
            obstacle_prediction_trajectory.trajectory_point(k)
                                          .trajectory_point()));
      }
      if (fabs(trajectory.back().first - start_point_timestamp_sec) <=
          FLAGS_trajectory_delta_t) {
        ADEBUG << "too short obstacle_prediction_trajectory. frame_num["
               << learning_data_frame->frame_num()
               << "] obstacle_id[" << obstacle_id << "] size["
               << trajectory.size() << "] timestamp_diff["
               << trajectory.back().first - start_point_timestamp_sec << "]";
        continue;
      }

      std::vector<TrajectoryPoint> evaluated_trajectory;
      EvaluateTrajectoryByTime(learning_data_frame->frame_num(),
                               trajectory,
                               start_point_timestamp_sec,
                               FLAGS_trajectory_delta_t,
                               &evaluated_trajectory);

      ADEBUG << "frame_num[" << learning_data_frame->frame_num()
             << "] obstacle_id[" << obstacle_id
             << "orig obstacle_prediction_trajectory[" << trajectory.size()
             << "] evaluated[" << evaluated_trajectory.size() << "]";

      // update learning_data
      learning_data_frame->mutable_obstacle(i)
                         ->mutable_obstacle_prediction()
                         ->mutable_trajectory(j)
                         ->clear_trajectory_point();
      for (const auto& tp : evaluated_trajectory) {
        auto obstacle_prediction_trajectory_point =
            learning_data_frame->mutable_obstacle(i)
                               ->mutable_obstacle_prediction()
                               ->mutable_trajectory(j)
                               ->add_trajectory_point();
        obstacle_prediction_trajectory_point->set_timestamp_sec(
            tp.timestamp_sec());
        obstacle_prediction_trajectory_point->mutable_trajectory_point()
                                            ->CopyFrom(tp.trajectory_point());
      }
    }
  }
}

}  // namespace planning
}  // namespace apollo
