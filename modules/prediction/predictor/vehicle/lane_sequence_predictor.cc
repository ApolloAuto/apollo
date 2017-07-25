/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/prediction/predictor/vehicle/lane_sequence_predictor.h"

#include <cmath>
#include "Eigen/Dense"

#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/common/prediction_map.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/log.h"

namespace apollo {
namespace prediction {

using ::apollo::common::PathPoint;
using ::apollo::common::TrajectoryPoint;
using ::apollo::common::math::KalmanFilter;

void LaneSequencePredictor::Predict(Obstacle* obstacle) {
  prediction_obstacle_.Clear();
  CHECK_NOTNULL(obstacle);
  CHECK_GT(obstacle->history_size(), 0);

  const Feature& feature = obstacle->latest_feature();
  if (!feature.has_lane() || !feature.lane().has_lane_graph()) {
    AERROR << "Obstacle [" << obstacle->id() << " has no lane graph.";
    return;
  }

  for (const auto& sequence : feature.lane().lane_graph().lane_sequence()) {
    if (sequence.lane_segment_size() == 0) {
      AERROR << "Empty lane segments.";
      continue;
    }

    if (!sequence.has_probability() ||
        sequence.probability() < FLAGS_lane_sequence_threshold) {
      AERROR << "Lane sequence [" << ToString(sequence)
             << "] has probability [" << sequence.probability()
             << "] less than threshold [" << FLAGS_lane_sequence_threshold
             << "]";
      continue;
    }

    std::string curr_lane_id = sequence.lane_segment(0).lane_id();
    std::vector<TrajectoryPoint> points;
    DrawLaneSequenceTrajectoryPoints(
        obstacle->kf_lane_tracker(curr_lane_id), sequence,
        FLAGS_prediction_duration, FLAGS_prediction_freq, &points);

    Trajectory trajectory;
    GenerateTrajectory(points, &trajectory);
    trajectory.set_probability(sequence.probability());
    prediction_obstacle_.set_predicted_period(FLAGS_prediction_duration);
    prediction_obstacle_.add_trajectory()->CopyFrom(trajectory);
  }
}

void LaneSequencePredictor::DrawLaneSequenceTrajectoryPoints(
    const KalmanFilter<double, 4, 2, 0>& kf,
    const LaneSequence& sequence,
    double total_time,
    double freq,
    std::vector<TrajectoryPoint> *points) {
  PredictionMap *map = PredictionMap::instance();
  CHECK_NOTNULL(map);

  Eigen::Matrix<double, 4, 1> state(kf.GetStateEstimate());
  double lane_s = state(0, 0);
  double lane_l = state(1, 0);
  double lane_speed = state(2, 0);
  double lane_acc = state(3, 0);

  Eigen::Matrix<double, 4, 4> transition(kf.GetTransitionMatrix());
  transition(0, 2) = freq;
  transition(0, 3) = 0.5 * freq * freq;
  transition(2, 3) = freq;

  int lane_segment_index = 0;
  std::string lane_id = sequence.lane_segment(lane_segment_index).lane_id();
  for (size_t i = 0; i < static_cast<size_t>(total_time / freq); ++i) {
    Eigen::Vector2d point;
    double theta = M_PI;
    if (map->SmoothPointFromLane(
        map->id(lane_id), lane_s, lane_l, &point, &theta) != 0) {
      AERROR << "Unable to get smooth point from lane [" << lane_id
             << "] with s [" << lane_s << "] and l [" << lane_l
             << "]";
      continue;
    }

    if (points->size() > 0) {
      PathPoint *prev_point = points->back().mutable_path_point();
      double x_diff = point.x() - prev_point->x();
      double y_diff = point.y() - prev_point->y();
      if (::apollo::common::math::DoubleCompare(x_diff, 0.0) != 0 ||
          ::apollo::common::math::DoubleCompare(y_diff, 0.0) != 0) {
        
        theta = std::atan2(y_diff, x_diff);
        prev_point->set_theta(theta);
      } else {
        theta = prev_point->theta();
      }
    }

    // add trajectory point
    TrajectoryPoint trajectory_point;
    PathPoint path_point;
    path_point.set_x(point.x());
    path_point.set_y(point.y());
    path_point.set_z(0.0);
    path_point.set_theta(theta);
    trajectory_point.mutable_path_point()->CopyFrom(path_point);
    trajectory_point.set_v(lane_speed);
    trajectory_point.set_a(lane_acc);
    trajectory_point.set_relative_time(static_cast<double>(i) * freq);
    points->emplace_back(std::move(trajectory_point));

    // update state
    if (::apollo::common::math::DoubleCompare(lane_speed, 0.0) <= 0) {
      lane_speed = 0.0;
      lane_acc = 0.0;
      transition(1, 1) = 1.0;
    } else if (::apollo::common::math::DoubleCompare(
        lane_speed, FLAGS_max_speed) >= 0) {
      lane_speed = FLAGS_max_speed;
      lane_acc = 0.0;
    }
    state(2, 0) = lane_speed;
    state(3, 0) = lane_acc;

    state = transition * state;
    if (::apollo::common::math::DoubleCompare(lane_s, state(0, 0)) >= 0) {
      state(0, 0) = lane_s;
      state(1, 0) = lane_l;
      state(2, 0) = 0.0;
      state(3, 0) = 0.0;
      transition(1, 1) = 1.0;
    }
    lane_s = state(0, 0);
    lane_l = state(1, 0);
    lane_speed = state(2, 0);
    lane_acc = state(3, 0);

    // find next lane id
    while (lane_s > map->LaneById(lane_id)->total_length() &&
           lane_segment_index < sequence.lane_segment_size()) {
      lane_s = lane_s - map->LaneById(lane_id)->total_length();
      lane_segment_index += 1;
      lane_id = sequence.lane_segment(lane_segment_index).lane_id();
    }
  }
}

std::string LaneSequencePredictor::ToString(const LaneSequence& sequence) {
  std::string str_lane_sequence;
  for (const auto& lane_segment: sequence.lane_segment()) {
    str_lane_sequence = str_lane_sequence + "->" + lane_segment.lane_id();
  }
  return str_lane_sequence;
}

}  // prediction
}  // apollo
