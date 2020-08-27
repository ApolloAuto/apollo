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

#include "modules/planning/common/trajectory_evaluator.h"

#include <algorithm>
#include <limits>

#include "absl/strings/str_cat.h"
#include "cyber/common/file.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/trajectory/discretized_trajectory.h"
#include "modules/planning/pipeline/evaluator_logger.h"

namespace apollo {
namespace planning {

void TrajectoryEvaluator::WriteLog(const std::string& msg) {
  AERROR << msg;
  if (FLAGS_planning_offline_learning) {
    EvaluatorLogger::GetStream() << msg << std::endl;
  }
}

void TrajectoryEvaluator::EvaluateTrajectoryByTime(
    const int frame_num, const std::string& obstacle_id,
    const std::vector<std::pair<double, CommonTrajectoryPointFeature>>&
        trajectory,
    const double start_point_timestamp_sec, const double delta_time,
    std::vector<TrajectoryPointFeature>* evaluated_trajectory) {
  if (trajectory.empty() || (fabs(trajectory.front().first -
                                  start_point_timestamp_sec) < delta_time &&
                             fabs(trajectory.back().first -
                                  start_point_timestamp_sec) < delta_time)) {
    return;
  }

  std::vector<apollo::common::TrajectoryPoint> updated_trajectory;
  for (const auto& tp : trajectory) {
    // CommonTrajectoryPointFeature => common::TrajectoryPoint
    double relative_time = tp.first - start_point_timestamp_sec;
    apollo::common::TrajectoryPoint trajectory_point;
    Convert(tp.second, relative_time, &trajectory_point);
    updated_trajectory.push_back(trajectory_point);
  }

  if (trajectory.front().first > trajectory.back().first) {
    std::reverse(updated_trajectory.begin(), updated_trajectory.end());
  }

  DiscretizedTrajectory discretized_trajectory;
  double last_relative_time = std::numeric_limits<double>::lowest();
  for (const auto& tp : updated_trajectory) {
    // filter out abnormal perception data
    if (tp.relative_time() > last_relative_time) {
      discretized_trajectory.AppendTrajectoryPoint(tp);
      last_relative_time = tp.relative_time();
    } else {
      const std::string msg = absl::StrCat(
          "DISCARD trajectory point: frame_num[", frame_num, "] obstacle_id[",
          obstacle_id, "] last_relative_time[", last_relative_time,
          "] relatice_time[", tp.relative_time(), "] relative_time_diff[",
          tp.relative_time() - last_relative_time, "]");
      WriteLog(msg);
    }
  }

  const int low_bound =
      std::max(-150.0,
               ceil(updated_trajectory.front().relative_time() / delta_time));
  const int high_bound =
      std::min(150.0,
               floor(updated_trajectory.back().relative_time() / delta_time));
  ADEBUG << "frame_num[" << frame_num << "] obstacle_id[" << obstacle_id
         << "] low[" << low_bound << "] high[" << high_bound << "]";
  for (int i = low_bound; i <= high_bound; ++i) {
    double timestamp_sec = start_point_timestamp_sec + i * delta_time;
    double relative_time = i * delta_time;
    auto tp = discretized_trajectory.Evaluate(relative_time);

    // common::TrajectoryPoint => TrajectoryPointFeature
    TrajectoryPointFeature trajectory_point;
    Convert(tp, timestamp_sec, &trajectory_point);

    evaluated_trajectory->push_back(trajectory_point);
  }
}

void TrajectoryEvaluator::EvaluateADCTrajectory(
    const double start_point_timestamp_sec, const double delta_time,
    LearningDataFrame* learning_data_frame) {
  std::vector<std::pair<double, CommonTrajectoryPointFeature>> trajectory;
  for (const auto& adc_tp : learning_data_frame->adc_trajectory_point()) {
    trajectory.push_back(
        std::make_pair(adc_tp.timestamp_sec(), adc_tp.trajectory_point()));
  }

  if (trajectory.size() <= 1) {
    const std::string msg = absl::StrCat(
        "too short adc_trajectory_point. frame_num[",
        learning_data_frame->frame_num(), "] size[", trajectory.size(), "]");
    WriteLog(msg);
    return;
  }

  learning_data_frame->clear_adc_trajectory_point();

  if (fabs(trajectory.front().first - start_point_timestamp_sec) <=
      delta_time) {
    auto adc_trajectory_point = learning_data_frame->add_adc_trajectory_point();
    adc_trajectory_point->set_timestamp_sec(trajectory.back().first);
    adc_trajectory_point->mutable_trajectory_point()->CopyFrom(
        trajectory.back().second);

    const std::string msg =
        absl::StrCat("too short adc_trajectory. frame_num[",
                     learning_data_frame->frame_num(), "] size[",
                     trajectory.size(), "] timestamp_diff[",
                     start_point_timestamp_sec - trajectory.front().first, "]");
    WriteLog(msg);
    return;
  }

  std::vector<TrajectoryPointFeature> evaluated_trajectory;
  EvaluateTrajectoryByTime(learning_data_frame->frame_num(), "adc_trajectory",
                           trajectory, start_point_timestamp_sec, delta_time,
                           &evaluated_trajectory);
  ADEBUG << "frame_num[" << learning_data_frame->frame_num()
         << "] orig adc_trajectory[" << trajectory.size()
         << "] evaluated_trajectory_size[" << evaluated_trajectory.size()
         << "]";

  for (const auto& tp : evaluated_trajectory) {
    if (tp.trajectory_point().relative_time() <= 0.0 &&
        tp.trajectory_point().relative_time() >=
            -FLAGS_trajectory_time_length) {
      auto adc_trajectory_point =
          learning_data_frame->add_adc_trajectory_point();
      adc_trajectory_point->set_timestamp_sec(tp.timestamp_sec());
      adc_trajectory_point->mutable_trajectory_point()->CopyFrom(
          tp.trajectory_point());
    } else {
      const std::string msg =
          absl::StrCat("DISCARD adc_trajectory_point. frame_num[",
                       learning_data_frame->frame_num(), "] size[",
                       evaluated_trajectory.size(), "] relative_time[",
                       tp.trajectory_point().relative_time(), "]");
      WriteLog(msg);
    }
  }
}

void TrajectoryEvaluator::EvaluateADCFutureTrajectory(
    const int frame_num,
    const std::vector<TrajectoryPointFeature>& adc_future_trajectory,
    const double start_point_timestamp_sec, const double delta_time,
    std::vector<TrajectoryPointFeature>* evaluated_adc_future_trajectory) {
  evaluated_adc_future_trajectory->clear();

  std::vector<std::pair<double, CommonTrajectoryPointFeature>> trajectory;
  for (const auto& adc_future_trajectory_point : adc_future_trajectory) {
    trajectory.push_back(
        std::make_pair(adc_future_trajectory_point.timestamp_sec(),
                       adc_future_trajectory_point.trajectory_point()));
  }

  if (trajectory.size() <= 1) {
    const std::string msg =
        absl::StrCat("too short adc_future_trajectory. frame_num[", frame_num,
                     "] size[", trajectory.size(), "]");
    WriteLog(msg);
    return;
  }

  if (fabs(trajectory.back().first - start_point_timestamp_sec) <= delta_time) {
    const std::string msg =
        absl::StrCat("too short adc_future_trajectory. frame_num[", frame_num,
                     "] size[", trajectory.size(), "] time_range[",
                     trajectory.back().first - start_point_timestamp_sec, "]");
    WriteLog(msg);
    return;
  }

  std::vector<TrajectoryPointFeature> evaluated_trajectory;
  EvaluateTrajectoryByTime(frame_num, "adc_future_trajectory", trajectory,
                           start_point_timestamp_sec, delta_time,
                           &evaluated_trajectory);

  ADEBUG << "frame_num[" << frame_num << "] orig adc_future_trajectory["
         << trajectory.size() << "] evaluated_trajectory_size["
         << evaluated_trajectory.size() << "]";

  if (evaluated_trajectory.empty()) {
    const std::string msg = absl::StrCat(
        "WARNING: adc_future_trajectory not long enough. ", "frame_num[",
        frame_num, "] size[", evaluated_trajectory.size(), "]");
    WriteLog(msg);
  } else {
    const double time_range =
        evaluated_trajectory.back().timestamp_sec() - start_point_timestamp_sec;
    if (time_range < FLAGS_trajectory_time_length) {
      const std::string msg = absl::StrCat(
          "WARNING: adc_future_trajectory not long enough. ", "frame_num[",
          frame_num, "] size[", evaluated_trajectory.size(), "] time_range[",
          time_range, "]");
      WriteLog(msg);
    }
  }

  for (const auto& tp : evaluated_trajectory) {
    if (tp.trajectory_point().relative_time() > 0.0 &&
        tp.trajectory_point().relative_time() <= FLAGS_trajectory_time_length) {
      evaluated_adc_future_trajectory->push_back(tp);
    } else {
      const std::string msg = absl::StrCat(
          "DISCARD adc_future_trajectory_point. frame_num[", frame_num,
          "] size[", evaluated_trajectory.size(), "] relative_time[",
          tp.trajectory_point().relative_time(), "]");
      WriteLog(msg);
    }
  }
}

void TrajectoryEvaluator::EvaluateObstacleTrajectory(
    const double start_point_timestamp_sec, const double delta_time,
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

      const double v = std::sqrt(perception_obstacle.velocity().x() *
                                     perception_obstacle.velocity().x() +
                                 perception_obstacle.velocity().y() *
                                     perception_obstacle.velocity().y());
      trajectory_point.set_v(v);

      const double a = std::sqrt(perception_obstacle.acceleration().x() *
                                     perception_obstacle.acceleration().x() +
                                 perception_obstacle.acceleration().y() *
                                     perception_obstacle.acceleration().y());
      trajectory_point.set_a(a);

      trajectory.push_back(std::make_pair(perception_obstacle.timestamp_sec(),
                                          trajectory_point));
    }

    std::vector<TrajectoryPointFeature> evaluated_trajectory;
    if (trajectory.size() == 1 ||
        fabs(trajectory.front().first - start_point_timestamp_sec)
            <= delta_time ||
        fabs(trajectory.front().first - trajectory.back().first)
            <= delta_time) {
      ADEBUG << "too short obstacle_trajectory. frame_num["
             << learning_data_frame->frame_num() << "] obstacle_id["
             << obstacle_id << "] size[" << trajectory.size()
             << "] timestamp_diff["
             << start_point_timestamp_sec - trajectory.front().first
             << "] time_range["
             << fabs(trajectory.front().first - trajectory.back().first) << "]";

      // pick at lease one point regardless of short timestamp,
      // to avoid model failure
      TrajectoryPointFeature trajectory_point;
      trajectory_point.set_timestamp_sec(trajectory.front().first);
      trajectory_point.mutable_trajectory_point()->CopyFrom(
          trajectory.front().second);
      evaluated_trajectory.push_back(trajectory_point);
    } else {
      EvaluateTrajectoryByTime(learning_data_frame->frame_num(),
                               std::to_string(obstacle_id), trajectory,
                               start_point_timestamp_sec, delta_time,
                               &evaluated_trajectory);

      ADEBUG << "frame_num[" << learning_data_frame->frame_num()
             << "] obstacle_id[" << obstacle_id
             << "] orig obstacle_trajectory[" << trajectory.size()
             << "] evaluated_trajectory_size[" << evaluated_trajectory.size()
             << "]";
    }

    // update learning_data
    learning_data_frame->mutable_obstacle(i)
        ->mutable_obstacle_trajectory()
        ->clear_evaluated_trajectory_point();
    for (const auto& tp : evaluated_trajectory) {
      auto evaluated_trajectory_point = learning_data_frame->mutable_obstacle(i)
                                            ->mutable_obstacle_trajectory()
                                            ->add_evaluated_trajectory_point();
      evaluated_trajectory_point->set_timestamp_sec(tp.timestamp_sec());
      evaluated_trajectory_point->mutable_trajectory_point()->CopyFrom(
          tp.trajectory_point());
    }
  }
}

void TrajectoryEvaluator::EvaluateObstaclePredictionTrajectory(
    const double start_point_timestamp_sec, const double delta_time,
    LearningDataFrame* learning_data_frame) {
  for (int i = 0; i < learning_data_frame->obstacle_size(); ++i) {
    const int obstacle_id = learning_data_frame->obstacle(i).id();
    const auto obstacle_prediction =
        learning_data_frame->obstacle(i).obstacle_prediction();
    for (int j = 0; j < obstacle_prediction.trajectory_size(); ++j) {
      const auto obstacle_prediction_trajectory =
          obstacle_prediction.trajectory(j);

      std::vector<std::pair<double, CommonTrajectoryPointFeature>> trajectory;
      for (const auto& trajectory_point :
           obstacle_prediction_trajectory.trajectory_point()) {
        const double timestamp_sec =
            obstacle_prediction.timestamp_sec() +
            trajectory_point.trajectory_point().relative_time();
        trajectory.push_back(
            std::make_pair(timestamp_sec, trajectory_point.trajectory_point()));
      }
      if (fabs(trajectory.back().first - start_point_timestamp_sec) <=
          delta_time) {
        ADEBUG << "too short obstacle_prediction_trajectory. frame_num["
               << learning_data_frame->frame_num() << "] obstacle_id["
               << obstacle_id << "] size[" << trajectory.size()
               << "] timestamp_diff["
               << trajectory.back().first - start_point_timestamp_sec << "]";
        continue;
      }

      std::vector<TrajectoryPointFeature> evaluated_trajectory;
      EvaluateTrajectoryByTime(learning_data_frame->frame_num(),
                               std::to_string(obstacle_id), trajectory,
                               start_point_timestamp_sec, delta_time,
                               &evaluated_trajectory);

      ADEBUG << "frame_num[" << learning_data_frame->frame_num()
             << "] obstacle_id[" << obstacle_id
             << "orig obstacle_prediction_trajectory[" << trajectory.size()
             << "] evaluated_trajectory_size[" << evaluated_trajectory.size()
             << "]";

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

void TrajectoryEvaluator::Convert(const CommonTrajectoryPointFeature& tp,
                                  const double relative_time,
                                  common::TrajectoryPoint* trajectory_point) {
  auto path_point = trajectory_point->mutable_path_point();
  path_point->set_x(tp.path_point().x());
  path_point->set_y(tp.path_point().y());
  path_point->set_z(tp.path_point().z());
  path_point->set_theta(tp.path_point().theta());
  path_point->set_s(tp.path_point().s());
  path_point->set_lane_id(tp.path_point().lane_id());
  trajectory_point->set_v(tp.v());
  trajectory_point->set_a(tp.a());
  trajectory_point->set_relative_time(relative_time);
  trajectory_point->mutable_gaussian_info()->CopyFrom(tp.gaussian_info());
}

void TrajectoryEvaluator::Convert(const common::TrajectoryPoint& tp,
                                  const double timestamp_sec,
                                  TrajectoryPointFeature* trajectory_point) {
  trajectory_point->set_timestamp_sec(timestamp_sec);
  auto path_point =
      trajectory_point->mutable_trajectory_point()->mutable_path_point();
  path_point->set_x(tp.path_point().x());
  path_point->set_y(tp.path_point().y());
  path_point->set_z(tp.path_point().z());
  path_point->set_theta(tp.path_point().theta());
  path_point->set_s(tp.path_point().s());
  path_point->set_lane_id(tp.path_point().lane_id());
  trajectory_point->mutable_trajectory_point()->set_v(tp.v());
  trajectory_point->mutable_trajectory_point()->set_a(tp.a());
  trajectory_point->mutable_trajectory_point()->set_relative_time(
      tp.relative_time());
  trajectory_point->mutable_trajectory_point()
      ->mutable_gaussian_info()
      ->CopyFrom(tp.gaussian_info());
}

}  // namespace planning
}  // namespace apollo
