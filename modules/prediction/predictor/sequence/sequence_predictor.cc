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

#include <cmath>
#include <limits>
#include <memory>
#include <utility>

#include "modules/common/adapters/proto/adapter_config.pb.h"

#include "modules/common/math/vec2d.h"
#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/common/prediction_map.h"
#include "modules/prediction/common/road_graph.h"
#include "modules/prediction/container/adc_trajectory/adc_trajectory_container.h"
#include "modules/prediction/container/container_manager.h"
#include "modules/prediction/container/pose/pose_container.h"

namespace apollo {
namespace prediction {

using ::apollo::common::PathPoint;
using ::apollo::common::TrajectoryPoint;
using ::apollo::common::adapter::AdapterConfig;
using ::apollo::common::math::Vec2d;
using ::apollo::hdmap::LaneInfo;

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
    if (distance < FLAGS_lane_change_dist) {
      (*enable_lane_sequence)[i] = false;
      ADEBUG << "Filter trajectory [" << ToString(sequence)
             << "] due to small distance " << distance << ".";
    }
  }

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

  for (int i = 0; i < num_lane_sequence; ++i) {
    const LaneSequence& sequence = lane_graph.lane_sequence(i);

    if (!(*enable_lane_sequence)[i]) {
      ADEBUG << "Disabled lane sequence [" << ToString(sequence) << "].";
      continue;
    }

    double probability = sequence.probability();
    if (probability < FLAGS_lane_sequence_threshold && i != all.first) {
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
    if (PredictionMap::IsLeftNeighborLane(
            PredictionMap::LaneById(lane_change_id),
            PredictionMap::LaneById(lane_id))) {
      return LaneChangeType::LEFT;
    } else if (PredictionMap::IsRightNeighborLane(
                   PredictionMap::LaneById(lane_change_id),
                   PredictionMap::LaneById(lane_id))) {
      return LaneChangeType::RIGHT;
    }
  }
  return LaneChangeType::INVALID;
}

double SequencePredictor::GetLaneChangeDistanceWithADC(
    const LaneSequence& lane_sequence) {
  PoseContainer* pose_container = dynamic_cast<PoseContainer*>(
      ContainerManager::instance()->GetContainer(AdapterConfig::LOCALIZATION));
  ADCTrajectoryContainer* adc_container = dynamic_cast<ADCTrajectoryContainer*>(
      ContainerManager::instance()->GetContainer(
          AdapterConfig::PLANNING_TRAJECTORY));
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
      return std::fabs(lane_s - obstacle_lane_s);
    }
  }

  ADEBUG << "Invalid ADC pose.";
  return std::numeric_limits<double>::max();
}

bool SequencePredictor::LaneSequenceWithMaxProb(const LaneChangeType& type,
                                                const double& probability,
                                                const double& max_prob) {
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
                                              const double& probability,
                                              const double& max_prob) {
  if (type == LaneChangeType::LEFT || type == LaneChangeType::RIGHT) {
    if (probability > max_prob) {
      return true;
    }
  }
  return false;
}

}  // namespace prediction
}  // namespace apollo
