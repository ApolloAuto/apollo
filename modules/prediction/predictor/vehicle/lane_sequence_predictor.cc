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
#include <utility>
#include <limits>

#include "modules/common/log.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/adapters/proto/adapter_config.pb.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/common/prediction_map.h"
#include "modules/prediction/container/container_manager.h"
#include "modules/prediction/container/obstacles/obstacles_container.h"
#include "modules/prediction/container/pose/pose_container.h"

namespace apollo {
namespace prediction {

using ::apollo::common::PathPoint;
using ::apollo::common::TrajectoryPoint;
using ::apollo::common::math::KalmanFilter;
using ::apollo::common::adapter::AdapterConfig;

void LaneSequencePredictor::Clear() {
  trajectories_.clear();
  adc_lane_id_.clear();
  adc_lane_s_ = 0.0;
}

void LaneSequencePredictor::Predict(Obstacle* obstacle) {
  Clear();

  CHECK_NOTNULL(obstacle);
  CHECK_GT(obstacle->history_size(), 0);

  const Feature& feature = obstacle->latest_feature();
  if (!feature.has_lane() || !feature.lane().has_lane_graph()) {
    AERROR << "Obstacle [" << obstacle->id() << " has no lane graph.";
    return;
  }

  std::string lane_id = "";
  if (feature.lane().has_lane_feature()) {
    lane_id = feature.lane().lane_feature().lane_id();
  }
  int num_lane_sequence = feature.lane().lane_graph().lane_sequence_size();
  std::vector<bool> enable_lane_sequence(num_lane_sequence, true);
  FilterLaneSequences(feature.lane().lane_graph(),
                      lane_id,
                      &enable_lane_sequence);

  for (int i = 0; i < num_lane_sequence; ++i) {
    const LaneSequence& sequence = feature.lane().lane_graph().lane_sequence(i);
    if (sequence.lane_segment_size() <= 0) {
      AERROR << "Empty lane segments.";
      continue;
    }

    if (!enable_lane_sequence[i]) {
      ADEBUG << "Lane sequence [" << ToString(sequence)
             << "] with probability ["
             <<sequence.probability() << "] is disqualified.";
      continue;
    }

    ADEBUG << "Obstacle [" << obstacle->id()
           << "] will draw a lane sequence trajectory ["
           << ToString(sequence) << "] with probability ["
           << sequence.probability() << "].";

    std::string curr_lane_id = sequence.lane_segment(0).lane_id();
    std::vector<TrajectoryPoint> points;
    DrawLaneSequenceTrajectoryPoints(
        obstacle->kf_lane_tracker(curr_lane_id), sequence,
        FLAGS_prediction_duration, FLAGS_prediction_freq, &points);

    Trajectory trajectory;
    GenerateTrajectory(points, &trajectory);
    trajectory.set_probability(sequence.probability());
    trajectories_.push_back(std::move(trajectory));
  }
  ADEBUG << "Obstacle [" << obstacle->id()
         << "] has total " << trajectories_.size()
         << " trajectories.";
}

void LaneSequencePredictor::FilterLaneSequences(
    const LaneGraph& lane_graph,
    const std::string& lane_id,
    std::vector<bool> *enable_lane_sequence) {
  int num_lane_sequence = lane_graph.lane_sequence_size();
  std::vector<int> lane_change_type(num_lane_sequence, -1);
  std::pair<int, double> change(-1, -1.0);
  std::pair<int, double> all(-1, -1.0);

  // Get ADC status
  GetADC();

  for (int i = 0; i < num_lane_sequence; ++i) {
    const LaneSequence& sequence = lane_graph.lane_sequence(i);

    // Get lane change type
    lane_change_type[i] = GetLaneChangeType(lane_id, sequence);

    double probability = sequence.probability();

    if (::apollo::common::math::DoubleCompare(probability, all.second) > 0 ||
        (::apollo::common::math::DoubleCompare(probability, all.second) == 0 &&
         lane_change_type[i] == 0)) {
      all.first = i;
      all.second = probability;
    }
    if (lane_change_type[i] > 0 &&
        ::apollo::common::math::DoubleCompare(probability, change.second) > 0) {
      change.first = i;
      change.second = probability;
    }
  }

  for (int i = 0; i < num_lane_sequence; ++i) {
    const LaneSequence& sequence = lane_graph.lane_sequence(i);

    // The obstacle has interference with ADC within a small distance
    if (::apollo::common::math::DoubleCompare(
        GetLaneChangeDistanceWithADC(sequence), FLAGS_lane_change_dist) < 0) {
      (*enable_lane_sequence)[i] = false;
      continue;
    }

    double probability = sequence.probability();
    if (::apollo::common::math::DoubleCompare(
        probability, FLAGS_lane_sequence_threshold) < 0 &&
        i != all.first) {
      (*enable_lane_sequence)[i] = false;
    } else if (change.first >= 0 &&
               change.first < num_lane_sequence &&
               lane_change_type[i] > 0 &&
               lane_change_type[i] != lane_change_type[change.first]) {
      (*enable_lane_sequence)[i] = false;
    }
  }
}

void LaneSequencePredictor::GetADC() {
  ObstaclesContainer *container = dynamic_cast<ObstaclesContainer*>(
      ContainerManager::instance()->GetContainer(
          AdapterConfig::PERCEPTION_OBSTACLES));
  if (container == nullptr) {
    AERROR << "Unavailable obstacle container";
    return;
  }

  Obstacle *adc = container->GetObstacle(PoseContainer::ID);
  if (adc != nullptr) {
    const Feature& feature = adc->latest_feature();
    if (feature.has_lane() &&
        feature.lane().has_lane_feature()) {
      adc_lane_id_ = feature.lane().lane_feature().lane_id();
      adc_lane_s_ = feature.lane().lane_feature().lane_s();
    }
    if (feature.has_position()) {
      adc_position_[0] = feature.position().x();
      adc_position_[1] = feature.position().y();
    }
  }
}

int LaneSequencePredictor::GetLaneChangeType(
    const std::string& lane_id,
    const LaneSequence& lane_sequence) {
  PredictionMap *map = PredictionMap::instance();

  std::string lane_change_id = lane_sequence.lane_segment(0).lane_id();
  if (lane_id == lane_change_id) {
    return 0;
  } else {
    if (map->IsLeftNeighborLane(
        map->LaneById(lane_change_id), map->LaneById(lane_id))) {
      return 1;
    } else if (map->IsRightNeighborLane(
        map->LaneById(lane_change_id), map->LaneById(lane_id))) {
      return 2;
    }
  }
  return -1;
}

double LaneSequencePredictor::GetLaneChangeDistanceWithADC(
    const LaneSequence& lane_sequence) {
  if (adc_lane_id_.empty() || lane_sequence.lane_segment_size() <= 0) {
    return std::numeric_limits<double>::max();
  }

  PredictionMap *map = PredictionMap::instance();
  std::string obstacle_lane_id = lane_sequence.lane_segment(0).lane_id();
  double obstacle_lane_s = lane_sequence.lane_segment(0).start_s();

  double lane_s = 0.0;
  double lane_l = 0.0;
  if (map->GetProjection(adc_position_,
                         map->LaneById(obstacle_lane_id),
                         &lane_s, &lane_l)) {
    return std::fabs(lane_s - obstacle_lane_s);
  }
  return std::numeric_limits<double>::max();
}

void LaneSequencePredictor::DrawLaneSequenceTrajectoryPoints(
    const KalmanFilter<double, 4, 2, 0>& kf,
    const LaneSequence& sequence,
    double total_time,
    double freq,
    std::vector<TrajectoryPoint> *points) {
  PredictionMap *map = PredictionMap::instance();

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
    if (!map->SmoothPointFromLane(lane_id, lane_s, lane_l, &point, &theta)) {
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
      AWARN << "Non-positive lane_speed tacked : " << lane_speed;
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
           lane_segment_index + 1 < sequence.lane_segment_size()) {
      lane_segment_index += 1;
      lane_s = lane_s - map->LaneById(lane_id)->total_length();
      state(0, 0) = lane_s;
      lane_id = sequence.lane_segment(lane_segment_index).lane_id();
    }
  }
}

std::string LaneSequencePredictor::ToString(const LaneSequence& sequence) {
  std::string str_lane_sequence = "";
  if (sequence.lane_segment_size() > 0) {
    str_lane_sequence += sequence.lane_segment(0).lane_id();
  }
  for (int i = 1; i < sequence.lane_segment_size(); ++i) {
    str_lane_sequence += ("->" + sequence.lane_segment(i).lane_id());
  }
  return str_lane_sequence;
}

}  // namespace prediction
}  // namespace apollo
