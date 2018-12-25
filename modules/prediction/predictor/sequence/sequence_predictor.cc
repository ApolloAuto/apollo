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

#include "modules/prediction/predictor/sequence/sequence_predictor.h"

#include <limits>
#include <memory>
#include <utility>

#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/container/container_manager.h"
#include "modules/prediction/container/pose/pose_container.h"

namespace apollo {
namespace prediction {

using apollo::common::PathPoint;
using apollo::common::TrajectoryPoint;
using apollo::common::adapter::AdapterConfig;
using apollo::hdmap::LaneInfo;

void SequencePredictor::Predict(Obstacle* obstacle) {
  Clear();

  CHECK_NOTNULL(obstacle);
  CHECK_GT(obstacle->history_size(), 0);
}

void SequencePredictor::Clear() { Predictor::Clear(); }

std::string SequencePredictor::ToString(const LaneSequence& sequence) {
  std::string str_lane_sequence = "";
  if (sequence.lane_segment_size() > 0) {
    str_lane_sequence += sequence.lane_segment(0).lane_id();
  }
  for (int i = 1; i < sequence.lane_segment_size(); ++i) {
    str_lane_sequence += ("->" + sequence.lane_segment(i).lane_id());
  }
  return str_lane_sequence;
}

void SequencePredictor::FilterLaneSequences(
    const Feature& feature, const std::string& lane_id,
    std::vector<bool>* enable_lane_sequence) {
  if (!feature.has_lane() || !feature.lane().has_lane_graph()) {
    return;
  }
  const LaneGraph& lane_graph = feature.lane().lane_graph();
  int num_lane_sequence = lane_graph.lane_sequence_size();
  std::vector<LaneChangeType> lane_change_type(num_lane_sequence,
                                               LaneChangeType::INVALID);
  std::pair<int, double> change(-1, -1.0);
  std::pair<int, double> all(-1, -1.0);

  /** 
    * Filter out those obstacles that are close to the ADC 
    * so that we will ignore them and drive normally unless
    * they really kick into our lane.
    */
  for (int i = 0; i < num_lane_sequence; ++i) {
    const LaneSequence& sequence = lane_graph.lane_sequence(i);
    lane_change_type[i] = GetLaneChangeType(lane_id, sequence);

    if (lane_change_type[i] != LaneChangeType::LEFT &&
        lane_change_type[i] != LaneChangeType::RIGHT &&
        lane_change_type[i] != LaneChangeType::ONTO_LANE) {
      ADEBUG "Ignore lane sequence [" << ToString(sequence) << "].";
      continue;
    }

    // The obstacle has interference with ADC within a small distance
    double distance = GetLaneChangeDistanceWithADC(sequence);
    ADEBUG << "Distance to ADC " << std::fixed << std::setprecision(6)
           << distance;
    if (distance > 0.0 && distance < FLAGS_lane_change_dist) {
      bool obs_within_its_own_lane = true;
      for (int j = 0; j < feature.polygon_point_size(); j ++) {
        Eigen::Vector2d position
            (feature.polygon_point(j).x(), feature.polygon_point(j).y());
        std::shared_ptr<const LaneInfo> lane_info =
            PredictionMap::LaneById(lane_id);
        if (lane_info == nullptr) {
          obs_within_its_own_lane = false;
          break;
        }

        double lane_s = 0.0;
        double lane_l = 0.0;
        PredictionMap::GetProjection(position, lane_info, &lane_s, &lane_l);

        double left = 0.0;
        double right = 0.0;
        lane_info->GetWidth(lane_s, &left, &right);

        if (lane_l > left || lane_l < -right) {
          obs_within_its_own_lane = false;
          break;
        }
      }

      if (obs_within_its_own_lane) {
        (*enable_lane_sequence)[i] = false;
        ADEBUG << "Filter trajectory [" << ToString(sequence)
               << "] due to small distance " << distance << ".";
      }
    }
  }

  /**
    * Pick the most probable lane-sequence and lane-change
    */
  for (int i = 0; i < num_lane_sequence; ++i) {
    const LaneSequence& sequence = lane_graph.lane_sequence(i);

    if (!(*enable_lane_sequence)[i]) {
      ADEBUG << "Disabled lane sequence [" << ToString(sequence) << "].";
      continue;
    }

    double probability = sequence.probability();
    if (LaneSequenceWithMaxProb(lane_change_type[i], probability, all.second)) {
      all.first = i;
      all.second = probability;
    }
    if (LaneChangeWithMaxProb(lane_change_type[i], probability,
                              change.second)) {
      change.first = i;
      change.second = probability;
    }
  }

  double lane_sequence_threshold = FLAGS_lane_sequence_threshold_cruise;
  if (feature.has_junction_feature()) {
    lane_sequence_threshold = FLAGS_lane_sequence_threshold_junction;
  }

  for (int i = 0; i < num_lane_sequence; ++i) {
    const LaneSequence& sequence = lane_graph.lane_sequence(i);

    if (!(*enable_lane_sequence)[i]) {
      ADEBUG << "Disabled lane sequence [" << ToString(sequence) << "].";
      continue;
    }

    double probability = sequence.probability();
    if (probability < lane_sequence_threshold && i != all.first) {
      (*enable_lane_sequence)[i] = false;
    } else if (change.first >= 0 && change.first < num_lane_sequence &&
               (lane_change_type[i] == LaneChangeType::LEFT ||
                lane_change_type[i] == LaneChangeType::RIGHT) &&
               lane_change_type[i] != lane_change_type[change.first]) {
      (*enable_lane_sequence)[i] = false;
    }
  }
}

SequencePredictor::LaneChangeType SequencePredictor::GetLaneChangeType(
    const std::string& lane_id, const LaneSequence& lane_sequence) {
  if (lane_id.empty()) {
    return LaneChangeType::ONTO_LANE;
  }

  std::string lane_change_id = lane_sequence.lane_segment(0).lane_id();
  if (lane_id == lane_change_id) {
    return LaneChangeType::STRAIGHT;
  } else {
    auto ptr_change_lane = PredictionMap::LaneById(lane_change_id);
    auto ptr_current_lane = PredictionMap::LaneById(lane_id);
    if (PredictionMap::IsLeftNeighborLane(ptr_change_lane, ptr_current_lane)) {
      return LaneChangeType::LEFT;
    } else if (PredictionMap::IsRightNeighborLane(ptr_change_lane,
        ptr_current_lane)) {
      return LaneChangeType::RIGHT;
    }
  }
  return LaneChangeType::INVALID;
}

double SequencePredictor::GetLaneChangeDistanceWithADC(
    const LaneSequence& lane_sequence) {
  auto pose_container =
      ContainerManager::Instance()->GetContainer<PoseContainer>(
          AdapterConfig::LOCALIZATION);
  auto adc_container = ContainerManager::Instance()->GetContainer<
      ADCTrajectoryContainer>(AdapterConfig::PLANNING_TRAJECTORY);

  CHECK_NOTNULL(pose_container);
  CHECK_NOTNULL(adc_container);

  if (!adc_container->HasOverlap(lane_sequence)) {
    ADEBUG << "The sequence [" << ToString(lane_sequence)
           << "] has no overlap with ADC.";
    return std::numeric_limits<double>::max();
  }

  Eigen::Vector2d adc_position;
  if (pose_container->ToPerceptionObstacle() != nullptr) {
    adc_position[0] = pose_container->ToPerceptionObstacle()->position().x();
    adc_position[1] = pose_container->ToPerceptionObstacle()->position().y();

    std::string obstacle_lane_id = lane_sequence.lane_segment(0).lane_id();
    double obstacle_lane_s = lane_sequence.lane_segment(0).start_s();
    double lane_s = 0.0;
    double lane_l = 0.0;
    if (PredictionMap::GetProjection(adc_position,
                                     PredictionMap::LaneById(obstacle_lane_id),
                                     &lane_s, &lane_l)) {
      ADEBUG << "Distance with ADC is " << std::fabs(lane_s - obstacle_lane_s);
      return obstacle_lane_s - lane_s;
    }
  }

  ADEBUG << "Invalid ADC pose.";
  return std::numeric_limits<double>::max();
}

bool SequencePredictor::LaneSequenceWithMaxProb(const LaneChangeType& type,
                                                const double probability,
                                                const double max_prob) {
  if (probability > max_prob) {
    return true;
  } else {
    double prob_diff = std::fabs(probability - max_prob);
    if (prob_diff <= std::numeric_limits<double>::epsilon() &&
        type == LaneChangeType::STRAIGHT) {
      return true;
    }
  }
  return false;
}

bool SequencePredictor::LaneChangeWithMaxProb(const LaneChangeType& type,
                                              const double probability,
                                              const double max_prob) {
  if (type == LaneChangeType::LEFT || type == LaneChangeType::RIGHT) {
    if (probability > max_prob) {
      return true;
    }
  }
  return false;
}

void SequencePredictor::DrawConstantAccelerationTrajectory(
    const Obstacle& obstacle, const LaneSequence& lane_sequence,
    const double total_time, const double period,
    const double acceleration,
    std::vector<TrajectoryPoint>* points) {
  const Feature& feature = obstacle.latest_feature();
  if (!feature.has_position() || !feature.has_velocity() ||
      !feature.position().has_x() || !feature.position().has_y()) {
    AERROR << "Obstacle [" << obstacle.id()
           << " is missing position or velocity";
    return;
  }

  Eigen::Vector2d position(feature.position().x(), feature.position().y());
  double speed = feature.speed();

  int lane_segment_index = 0;
  std::string lane_id =
      lane_sequence.lane_segment(lane_segment_index).lane_id();
  std::shared_ptr<const LaneInfo> lane_info = PredictionMap::LaneById(lane_id);
  double lane_s = 0.0;
  double lane_l = 0.0;
  if (!PredictionMap::GetProjection(position, lane_info, &lane_s, &lane_l)) {
    AERROR << "Failed in getting lane s and lane l";
    return;
  }
  size_t total_num = static_cast<size_t>(total_time / period);
  for (size_t i = 0; i < total_num; ++i) {
    double relative_time = static_cast<double>(i) * period;
    Eigen::Vector2d point;
    double theta = M_PI;
    if (!PredictionMap::SmoothPointFromLane(lane_id, lane_s, lane_l, &point,
                                            &theta)) {
      AERROR << "Unable to get smooth point from lane [" << lane_id
             << "] with s [" << lane_s << "] and l [" << lane_l << "]";
      break;
    }
    TrajectoryPoint trajectory_point;
    PathPoint path_point;
    path_point.set_x(point.x());
    path_point.set_y(point.y());
    path_point.set_z(0.0);
    path_point.set_theta(theta);
    path_point.set_lane_id(lane_id);
    trajectory_point.mutable_path_point()->CopyFrom(path_point);
    trajectory_point.set_v(speed);
    trajectory_point.set_a(0.0);
    trajectory_point.set_relative_time(relative_time);
    points->emplace_back(std::move(trajectory_point));

    if (speed < FLAGS_double_precision) {
      continue;
    }

    lane_s += speed * period + 0.5 * acceleration * period * period;
    speed += acceleration * period;

    while (lane_s > PredictionMap::LaneById(lane_id)->total_length() &&
           lane_segment_index + 1 < lane_sequence.lane_segment_size()) {
      lane_segment_index += 1;
      lane_s = lane_s - PredictionMap::LaneById(lane_id)->total_length();
      lane_id = lane_sequence.lane_segment(lane_segment_index).lane_id();
    }

    lane_l *= FLAGS_go_approach_rate;
  }
}

}  // namespace prediction
}  // namespace apollo
