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

#include "modules/prediction/predictor/lane_sequence/lane_sequence_predictor.h"

#include <utility>
#include <string>

#include "modules/common/log.h"
#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/common/prediction_util.h"

namespace apollo {
namespace prediction {

using apollo::common::PathPoint;
using apollo::common::TrajectoryPoint;
using apollo::common::math::KalmanFilter;

void LaneSequencePredictor::Predict(Obstacle* obstacle) {
  Clear();

  CHECK_NOTNULL(obstacle);
  CHECK_GT(obstacle->history_size(), 0);

  const Feature& feature = obstacle->latest_feature();
  if (!feature.has_lane() || !feature.lane().has_lane_graph()) {
    AERROR << "Obstacle [" << obstacle->id() << " has no lane graph.";
    return;
  }

  std::string lane_id = feature.lane().lane_feature().lane_id();
  int num_lane_sequence = feature.lane().lane_graph().lane_sequence_size();
  std::vector<bool> enable_lane_sequence(num_lane_sequence, true);
  FilterLaneSequences(feature.lane().lane_graph(), lane_id,
                      &enable_lane_sequence);

  for (int i = 0; i < num_lane_sequence; ++i) {
    const LaneSequence& sequence = feature.lane().lane_graph().lane_sequence(i);
    if (sequence.lane_segment_size() <= 0) {
      AERROR << "Empty lane segments.";
      continue;
    }

    if (!enable_lane_sequence[i]) {
      ADEBUG << "Lane sequence [" << ToString(sequence)
             << "] with probability [" << sequence.probability()
             << "] is disqualified.";
      continue;
    }

    ADEBUG << "Obstacle [" << obstacle->id()
           << "] will draw a lane sequence trajectory [" << ToString(sequence)
           << "] with probability [" << sequence.probability() << "].";

    std::string curr_lane_id = sequence.lane_segment(0).lane_id();
    std::vector<TrajectoryPoint> points;
    DrawLaneSequenceTrajectoryPoints(obstacle->kf_lane_tracker(curr_lane_id),
                                     sequence, FLAGS_prediction_duration,
                                     FLAGS_prediction_freq, &points);

    Trajectory trajectory = GenerateTrajectory(points);
    trajectory.set_probability(sequence.probability());
    trajectories_.push_back(std::move(trajectory));
  }

  ADEBUG << "Obstacle [" << obstacle->id() << "] has total "
         << trajectories_.size() << " trajectories.";
}

void LaneSequencePredictor::DrawLaneSequenceTrajectoryPoints(
    const KalmanFilter<double, 4, 2, 0>& kf, const LaneSequence& sequence,
    double total_time, double freq, std::vector<TrajectoryPoint>* points) {
  // PredictionMap* map = PredictionMap::instance();

  Eigen::Matrix<double, 4, 1> state(kf.GetStateEstimate());
  Eigen::Matrix<double, 4, 4> transition(kf.GetTransitionMatrix());
  transition(0, 2) = freq;
  transition(0, 3) = 0.5 * freq * freq;
  transition(2, 3) = freq;

  size_t num = static_cast<size_t>(total_time / freq);
  ::apollo::prediction::predictor_util::GenerateLaneSequenceTrajectoryPoints(
      &state, &transition, sequence, num, freq, points);
}

}  // namespace prediction
}  // namespace apollo
