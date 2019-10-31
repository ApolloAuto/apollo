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

#include "modules/prediction/predictor/extrapolation/extrapolation_predictor.h"
#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/common/prediction_map.h"

namespace apollo {
namespace prediction {

using apollo::common::PathPoint;
using apollo::common::TrajectoryPoint;

ExtrapolationPredictor::ExtrapolationPredictor() {
  predictor_type_ = ObstacleConf::EXTRAPOLATION_PREDICTOR;
}

void ExtrapolationPredictor::Predict(
    const ADCTrajectoryContainer* adc_trajectory_container, Obstacle* obstacle,
    ObstaclesContainer* obstacles_container) {
  Clear();

  CHECK_NOTNULL(obstacle);
  CHECK_GT(obstacle->history_size(), 0);

  obstacle->SetPredictorType(predictor_type_);

  Feature* feature_ptr = obstacle->mutable_latest_feature();

  if (!feature_ptr->lane().has_lane_graph()) {
    AERROR << "Obstacle [" << obstacle->id() << "] has no lane graph.";
    return;
  }
  for (int i = 0; i < feature_ptr->predicted_trajectory_size(); ++i) {
    Trajectory* trajectory_ptr = feature_ptr->mutable_predicted_trajectory(i);
    PostProcess(trajectory_ptr);
  }
}

void ExtrapolationPredictor::PostProcess(Trajectory* trajectory_ptr) {
  // TODO(kechxu) handle corner cases
  constexpr int kNumTailPoint = 5;
  ExtrapolationPredictor::LaneSearchResult
  lane_search_result = SearchExtrapolationLane(*trajectory_ptr, kNumTailPoint);
  if (lane_search_result.found) {
    ExtrapolateByLane(lane_search_result.lane_id, trajectory_ptr);
  } else {
    ExtrapolateByFreeMove(kNumTailPoint, trajectory_ptr);
  }
}

ExtrapolationPredictor::LaneSearchResult
ExtrapolationPredictor::SearchExtrapolationLane(
    const Trajectory& trajectory, const int num_tail_point) {
  constexpr double radius = 1.0;
  constexpr double angle_diff_threshold = M_PI / 3.0;
  int num_trajectory_point = trajectory.trajectory_point_size();

  LaneSearchResult lane_search_result;
  for (int i = num_trajectory_point - 1;
       i >= num_trajectory_point - num_tail_point; --i) {
    const TrajectoryPoint& trajectory_point = trajectory.trajectory_point(i);
    const PathPoint& path_point = trajectory_point.path_point();
    apollo::common::PointENU point_enu;
    point_enu.set_x(path_point.x());
    point_enu.set_y(path_point.y());
    double heading = path_point.theta();
    auto lane_info_ptr = PredictionMap::GetMostLikelyCurrentLane(
        point_enu, radius, heading, angle_diff_threshold);
    if (lane_info_ptr != nullptr) {
      lane_search_result.found = true;
      lane_search_result.lane_id = lane_info_ptr->lane().id().id();
      lane_search_result.point_index = i;
      break;
    }
  }
  return lane_search_result;
}

void ExtrapolationPredictor::ExtrapolateByLane(
    const std::string& lane_id, Trajectory* trajectory_ptr) {
  // TODO(kechxu) implement
}

void ExtrapolationPredictor::ExtrapolateByFreeMove(
    const int num_tail_point, Trajectory* trajectory_ptr) {
  int num_trajectory_point = trajectory_ptr->trajectory_point_size();
  CHECK_GT(num_trajectory_point, num_tail_point);
  double time_resolution = FLAGS_prediction_trajectory_time_resolution;
  double time_length = FLAGS_prediction_trajectory_time_length;
  const TrajectoryPoint& last_point =
      trajectory_ptr->trajectory_point(num_trajectory_point - 1);
  const TrajectoryPoint& mid_point =
      trajectory_ptr->trajectory_point(num_trajectory_point - num_tail_point);
  double theta = last_point.path_point().theta();
  double diff_x = last_point.path_point().x() - mid_point.path_point().x();
  double diff_y = last_point.path_point().y() - mid_point.path_point().y();
  double speed = std::hypot(diff_x, diff_y) /
                 (num_tail_point * time_resolution);
  double relative_time = last_point.relative_time() + time_resolution;
  while (relative_time < time_length) {
    int prev_size = trajectory_ptr->trajectory_point_size();
    const TrajectoryPoint& prev_point =
        trajectory_ptr->trajectory_point(prev_size);
    TrajectoryPoint* curr_point = trajectory_ptr->add_trajectory_point();
    double dx = time_resolution * speed * std::cos(theta);
    double dy = time_resolution * speed * std::sin(theta);
    double curr_x = prev_point.path_point().x() + dx;
    double curr_y = prev_point.path_point().y() + dy;
    PathPoint* curr_path_point_ptr = curr_point->mutable_path_point();
    curr_path_point_ptr->set_x(curr_x);
    curr_path_point_ptr->set_y(curr_y);
    curr_path_point_ptr->set_theta(theta);
    curr_point->set_v(speed);
    curr_point->set_relative_time(relative_time);

    relative_time += time_resolution;
  }
}

}  // namespace prediction
}  // namespace apollo
