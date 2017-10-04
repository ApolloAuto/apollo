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

#include "modules/prediction/predictor/move_sequence/move_sequence_predictor.h"

#include <cmath>
#include <utility>
#include <limits>
#include <memory>

#include "Eigen/Dense"
#include "modules/common/adapters/proto/adapter_config.pb.h"
#include "modules/common/log.h"
#include "modules/common/math/math_utils.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/common/prediction_util.h"
#include "modules/prediction/common/prediction_map.h"
#include "modules/prediction/common/road_graph.h"
#include "modules/prediction/container/container_manager.h"
#include "modules/prediction/container/obstacles/obstacles_container.h"
#include "modules/prediction/container/pose/pose_container.h"

namespace apollo {
namespace prediction {

using apollo::common::PathPoint;
using apollo::common::TrajectoryPoint;
using apollo::common::math::KalmanFilter;
using apollo::common::adapter::AdapterConfig;
using apollo::hdmap::LaneInfo;

namespace {

void WeightedMean(
    const TrajectoryPoint& point1,
    const TrajectoryPoint& point2,
    const double weight1, const double weight2,
    TrajectoryPoint* ret_point) {
  // TODO(all) Double check if the following CHECK is okay.
  CHECK_EQ(point1.relative_time(), point2.relative_time());

  double ret_x = weight1 * point1.path_point().x() +
                 weight2 * point2.path_point().x();
  double ret_y = weight1 * point1.path_point().y() +
                 weight2 * point2.path_point().y();
  double ret_z = 0.0;
  double ret_theta = weight1 * point1.path_point().theta() +
                     weight2 * point2.path_point().theta();
  double ret_v = weight1 * point1.v() + weight2 * point2.v();
  double ret_a = weight1 * point1.a() + weight2 * point2.a();
  double ret_relative_time = point1.relative_time();

  ret_point->mutable_path_point()->set_x(ret_x);
  ret_point->mutable_path_point()->set_y(ret_y);
  ret_point->mutable_path_point()->set_z(ret_z);
  ret_point->mutable_path_point()->set_theta(ret_theta);
  ret_point->set_v(ret_v);
  ret_point->set_a(ret_a);
  ret_point->set_relative_time(ret_relative_time);
}

}  // namespace

void MoveSequencePredictor::Predict(Obstacle* obstacle) {
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
    DrawLaneSequenceTrajectoryPoints(*obstacle, sequence,
        FLAGS_prediction_duration, FLAGS_prediction_freq, &points);

    Trajectory trajectory = GenerateTrajectory(points);
    trajectory.set_probability(sequence.probability());
    trajectories_.push_back(std::move(trajectory));
  }
  ADEBUG << "Obstacle [" << obstacle->id() << "] has total "
         << trajectories_.size() << " trajectories.";
}

void MoveSequencePredictor::DrawLaneSequenceTrajectoryPoints(
    const Obstacle& obstacle,
    const LaneSequence& lane_sequence,
    const double total_time, const double freq,
    std::vector<TrajectoryPoint>* points) {

  points->clear();
  std::vector<TrajectoryPoint> maneuver_trajectory_points;
  std::vector<TrajectoryPoint> motion_trajectory_points;
  DrawManeuverTrajectoryPoints(obstacle, lane_sequence, total_time, freq,
      &maneuver_trajectory_points);
  DrawMotionTrajectoryPoints(obstacle, total_time, freq,
      &motion_trajectory_points);
  CHECK_EQ(maneuver_trajectory_points.size(),
           motion_trajectory_points.size());
  double t = 0.0;
  for (size_t i = 0; i < maneuver_trajectory_points.size(); ++i) {
    double motion_weight = MotionWeight(t);
    const TrajectoryPoint& maneuver_point = maneuver_trajectory_points[i];
    const TrajectoryPoint& motion_point = motion_trajectory_points[i];
    TrajectoryPoint trajectory_point;
    WeightedMean(maneuver_point, motion_point,
        1 - motion_weight, motion_weight, &trajectory_point);
    points->push_back(trajectory_point);
    t += freq;
  }
}

void MoveSequencePredictor::DrawManeuverTrajectoryPoints(
    const Obstacle& obstacle,
    const LaneSequence& lane_sequence,
    const double total_time, const double freq,
    std::vector<TrajectoryPoint>* points) {
  // TODO(kechxu) implement
}

void MoveSequencePredictor::DrawMotionTrajectoryPoints(
    const Obstacle& obstacle,
    const double total_time, const double freq,
    std::vector<TrajectoryPoint>* points) {

  // Apply free_move here
  const Feature& feature = obstacle.latest_feature();
  if (!feature.has_position() || !feature.has_velocity() ||
      !feature.position().has_x() || !feature.position().has_y()) {
    AERROR << "Obstacle [" << obstacle.id()
           << " is missing position or velocity";
    return;
  }

  Eigen::Vector2d position(feature.position().x(), feature.position().y());
  Eigen::Vector2d velocity(feature.velocity().x(), feature.velocity().y());
  Eigen::Vector2d acc(feature.acceleration().x(), feature.acceleration().y());
  if (FLAGS_enable_kf_tracking) {
    position(0) = feature.t_position().x();
    position(1) = feature.t_position().y();
    velocity(0) = feature.t_velocity().x();
    velocity(1) = feature.t_velocity().y();
    acc(0) = feature.t_acceleration().x();
    acc(1) = feature.t_acceleration().y();
  }
  const KalmanFilter<double, 6, 2, 0>& kf = obstacle.kf_motion_tracker();

  Eigen::Matrix<double, 6, 1> state(kf.GetStateEstimate());
  state(0, 0) = 0.0;
  state(1, 0) = 0.0;
  state(2, 0) = velocity(0);
  state(3, 0) = velocity(1);
  state(4, 0) = common::math::Clamp(acc(0), FLAGS_min_acc, FLAGS_max_acc);
  state(5, 0) = common::math::Clamp(acc(1), FLAGS_min_acc, FLAGS_max_acc);

  Eigen::Matrix<double, 6, 6> transition(kf.GetTransitionMatrix());
  transition(0, 2) = freq;
  transition(0, 4) = 0.5 * freq * freq;
  transition(1, 3) = freq;
  transition(1, 5) = 0.5 * freq * freq;
  transition(2, 4) = freq;
  transition(3, 5) = freq;

  size_t num = static_cast<size_t>(total_time / freq);
  apollo::prediction::predictor_util::GenerateFreeMoveTrajectoryPoints(
      &state, transition, num, freq, points);

  for (size_t i = 0; i < points->size(); ++i) {
    apollo::prediction::predictor_util::TranslatePoint(
        position[0], position[1], &(points->operator[](i)));
  }
}

double MoveSequencePredictor::Cost(const double t,
    const std::array<double, COEFF_SIZE>& coeffs,
    const double alpha) {
  // TODO(kechxu) implement
  return 0.0;
}

double MoveSequencePredictor::MotionWeight(const double t) {
  // TODO(kechxu) Avoid the following hard-coded constants
  double a = 1.2;
  double b = 5.0;
  double c = 1.5;

  return 1.0 - 1.0 / (1.0 + a * std::exp(-b * (t - c)));
}

void MoveSequencePredictor::FilterLaneSequences(
    const LaneGraph& lane_graph, const std::string& lane_id,
    std::vector<bool>* enable_lane_sequence) {
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

    if (probability > all.second ||
        (probability == all.second && lane_change_type[i] == 0)) {
      all.first = i;
      all.second = probability;
    }
    if (lane_change_type[i] > 0 && probability > change.second) {
      change.first = i;
      change.second = probability;
    }
  }

  for (int i = 0; i < num_lane_sequence; ++i) {
    const LaneSequence& sequence = lane_graph.lane_sequence(i);

    // The obstacle has interference with ADC within a small distance
    if (GetLaneChangeDistanceWithADC(sequence) < FLAGS_lane_change_dist) {
      (*enable_lane_sequence)[i] = false;
      continue;
    }

    double probability = sequence.probability();
    if (probability < FLAGS_lane_sequence_threshold && i != all.first) {
      (*enable_lane_sequence)[i] = false;
    } else if (change.first >= 0 && change.first < num_lane_sequence &&
               lane_change_type[i] > 0 &&
               lane_change_type[i] != lane_change_type[change.first]) {
      (*enable_lane_sequence)[i] = false;
    }
  }
}

void MoveSequencePredictor::GetADC() {
  ObstaclesContainer* container = dynamic_cast<ObstaclesContainer*>(
      ContainerManager::instance()->GetContainer(
          AdapterConfig::PERCEPTION_OBSTACLES));
  if (container == nullptr) {
    AERROR << "Unavailable obstacle container";
    return;
  }

  Obstacle* adc = container->GetObstacle(PoseContainer::ID);
  if (adc != nullptr) {
    const Feature& feature = adc->latest_feature();
    if (feature.has_lane() && feature.lane().has_lane_feature()) {
      adc_lane_id_ = feature.lane().lane_feature().lane_id();
      adc_lane_s_ = feature.lane().lane_feature().lane_s();
    }
    if (feature.has_position()) {
      adc_position_[0] = feature.position().x();
      adc_position_[1] = feature.position().y();
    }
  }
}

int MoveSequencePredictor::GetLaneChangeType(
    const std::string& lane_id, const LaneSequence& lane_sequence) {
  PredictionMap* map = PredictionMap::instance();

  std::string lane_change_id = lane_sequence.lane_segment(0).lane_id();
  if (lane_id == lane_change_id) {
    return 0;
  } else {
    if (map->IsLeftNeighborLane(map->LaneById(lane_change_id),
                                map->LaneById(lane_id))) {
      return 1;
    } else if (map->IsRightNeighborLane(map->LaneById(lane_change_id),
                                        map->LaneById(lane_id))) {
      return 2;
    }
  }
  return -1;
}

double MoveSequencePredictor::GetLaneChangeDistanceWithADC(
    const LaneSequence& lane_sequence) {
  if (adc_lane_id_.empty() || lane_sequence.lane_segment_size() <= 0) {
    return std::numeric_limits<double>::max();
  }

  PredictionMap* map = PredictionMap::instance();
  std::string obstacle_lane_id = lane_sequence.lane_segment(0).lane_id();
  double obstacle_lane_s = lane_sequence.lane_segment(0).start_s();

  if (SameLaneSequence(obstacle_lane_id, obstacle_lane_s)) {
    return std::numeric_limits<double>::max();
  }

  double lane_s = 0.0;
  double lane_l = 0.0;
  if (map->GetProjection(adc_position_, map->LaneById(obstacle_lane_id),
                         &lane_s, &lane_l)) {
    return std::fabs(lane_s - obstacle_lane_s);
  }
  return std::numeric_limits<double>::max();
}

bool MoveSequencePredictor::SameLaneSequence(const std::string& lane_id,
                                             double lane_s) {
  PredictionMap* map = PredictionMap::instance();

  std::shared_ptr<const LaneInfo> obstacle_lane = map->LaneById(lane_id);
  std::shared_ptr<const LaneInfo> adc_lane = map->LaneById(adc_lane_id_);

  if (obstacle_lane != nullptr && adc_lane != nullptr) {
    RoadGraph obstacle_road_graph(lane_s, FLAGS_lane_change_dist,
                                  obstacle_lane);
    LaneGraph obstacle_lane_graph;
    obstacle_road_graph.BuildLaneGraph(&obstacle_lane_graph);

    RoadGraph adc_road_graph(adc_lane_s_, FLAGS_lane_change_dist, adc_lane);
    LaneGraph adc_lane_graph;
    adc_road_graph.BuildLaneGraph(&adc_lane_graph);

    return obstacle_road_graph.IsOnLaneGraph(adc_lane, obstacle_lane_graph) ||
           adc_road_graph.IsOnLaneGraph(obstacle_lane, adc_lane_graph);
  }

  return false;
}

std::string MoveSequencePredictor::ToString(const LaneSequence& sequence) {
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
