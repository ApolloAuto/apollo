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
#pragma once

#include <string>
#include <utility>
#include <vector>

#include "modules/planning/planning_base/proto/learning_data.pb.h"

namespace apollo {
namespace planning {

class TrajectoryEvaluator {
 public:
  ~TrajectoryEvaluator() = default;

  void EvaluateADCTrajectory(const double start_point_timestamp_sec,
                             const double delta_time,
                             LearningDataFrame* learning_data_frame);

  void EvaluateADCFutureTrajectory(
      const int frame_num,
      const std::vector<TrajectoryPointFeature>& adc_future_trajectory,
      const double start_point_timestamp_sec, const double delta_time,
      std::vector<TrajectoryPointFeature>* evaluated_adc_future_trajectory);

  void EvaluateObstacleTrajectory(const double start_point_timestamp_sec,
                                  const double delta_time,
                                  LearningDataFrame* learning_data_frame);

  void EvaluateObstaclePredictionTrajectory(
      const double start_point_timestamp_sec, const double delta_time,
      LearningDataFrame* learning_data_frame);

 private:
  void EvaluateTrajectoryByTime(
      const int frame_num, const std::string& obstacle_id,
      const std::vector<std::pair<double, CommonTrajectoryPointFeature>>&
          trajectory,
      const double start_point_timestamp_sec, const double delta_time,
      std::vector<TrajectoryPointFeature>* evaluated_trajectory);

  void Convert(const CommonTrajectoryPointFeature& tp,
               const double relative_time,
               common::TrajectoryPoint* trajectory_point);
  void Convert(const common::TrajectoryPoint& tp, const double timestamp_sec,
               TrajectoryPointFeature* trajectory_point);

  void WriteLog(const std::string& msg);
};

}  // namespace planning
}  // namespace apollo
