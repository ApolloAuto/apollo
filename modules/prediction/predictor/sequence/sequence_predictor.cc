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
#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/common/prediction_map.h"
#include "modules/prediction/common/road_graph.h"
#include "modules/prediction/container/container_manager.h"
#include "modules/prediction/container/obstacles/obstacles_container.h"
#include "modules/prediction/container/pose/pose_container.h"

namespace apollo {
namespace prediction {

using apollo::common::adapter::AdapterConfig;
using apollo::hdmap::LaneInfo;
using apollo::common::TrajectoryPoint;
using apollo::common::PathPoint;

void SequencePredictor::Predict(Obstacle* obstacle) {
  Clear();

  CHECK_NOTNULL(obstacle);
  CHECK_GT(obstacle->history_size(), 0);
}

void SequencePredictor::Clear() {
  Predictor::Clear();
  adc_lane_id_.clear();
  adc_lane_s_ = 0.0;
}

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
    const LaneGraph& lane_graph, const std::string& lane_id,
    std::vector<bool>* enable_lane_sequence) {
  int num_lane_sequence = lane_graph.lane_sequence_size();
  std::vector<LaneChangeType> lane_change_type(num_lane_sequence,
                                               LaneChangeType::INVALID);
  std::pair<int, double> change(-1, -1.0);
  std::pair<int, double> all(-1, -1.0);

  // Get ADC status
  GetADC();

  for (int i = 0; i < num_lane_sequence; ++i) {
    const LaneSequence& sequence = lane_graph.lane_sequence(i);

    lane_change_type[i] = GetLaneChangeType(lane_id, sequence);
    if (lane_change_type[i] == LaneChangeType::INVALID) {
      ADEBUG "Invalid lane change type for lane sequence ["
          << ToString(sequence) << "].";
      continue;
    }

    double distance = GetLaneChangeDistanceWithADC(sequence);
    ADEBUG << "Distance to ADC: " << distance << " " << ToString(sequence);
    // The obstacle has interference with ADC within a small distance
    if (distance < FLAGS_lane_change_dist &&
        (lane_change_type[i] == LaneChangeType::LEFT ||
         lane_change_type[i] == LaneChangeType::RIGHT)) {
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

void SequencePredictor::GetADC() {
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
      ADEBUG << "ADC: lane_id: " << adc_lane_id_ << ", s: " << adc_lane_s_;
    }
    if (feature.has_position()) {
      adc_position_[0] = feature.position().x();
      adc_position_[1] = feature.position().y();
    }
  }
}

SequencePredictor::LaneChangeType SequencePredictor::GetLaneChangeType(
    const std::string& lane_id, const LaneSequence& lane_sequence) {
  PredictionMap* map = PredictionMap::instance();

  std::string lane_change_id = lane_sequence.lane_segment(0).lane_id();
  if (lane_id == lane_change_id) {
    return LaneChangeType::STRAIGHT;
  } else {
    if (map->IsLeftNeighborLane(map->LaneById(lane_change_id),
                                map->LaneById(lane_id))) {
      return LaneChangeType::LEFT;
    } else if (map->IsRightNeighborLane(map->LaneById(lane_change_id),
                                        map->LaneById(lane_id))) {
      return LaneChangeType::RIGHT;
    }
  }
  return LaneChangeType::INVALID;
}

double SequencePredictor::GetLaneChangeDistanceWithADC(
    const LaneSequence& lane_sequence) {
  if (adc_lane_id_.empty() || lane_sequence.lane_segment_size() <= 0) {
    return std::numeric_limits<double>::max();
  }

  PredictionMap* map = PredictionMap::instance();
  std::string obstacle_lane_id = lane_sequence.lane_segment(0).lane_id();
  double obstacle_lane_s = lane_sequence.lane_segment(0).start_s();

  if (!SameLaneSequence(obstacle_lane_id, obstacle_lane_s)) {
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

bool SequencePredictor::SameLaneSequence(const std::string& lane_id,
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

bool SequencePredictor::LaneSequenceWithMaxProb(const LaneChangeType& type,
                                                const double& probability,
                                                const double& max_prob) {
  if (probability > max_prob) {
    return true;
  } else {
    double prob_diff = std::abs(probability - max_prob);
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
