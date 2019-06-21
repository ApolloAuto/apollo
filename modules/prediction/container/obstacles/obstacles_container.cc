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

#include "modules/prediction/container/obstacles/obstacles_container.h"

#include <unordered_set>
#include <utility>

#include "modules/prediction/common/feature_output.h"
#include "modules/prediction/common/junction_analyzer.h"
#include "modules/prediction/common/prediction_constants.h"
#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/common/prediction_system_gflags.h"
#include "modules/prediction/container/obstacles/obstacle_clusters.h"

namespace apollo {
namespace prediction {

using apollo::perception::PerceptionObstacle;
using apollo::perception::PerceptionObstacles;
using apollo::prediction::PredictionConstants;

ObstaclesContainer::ObstaclesContainer()
    : ptr_obstacles_(FLAGS_max_num_obstacles),
      id_mapping_(FLAGS_max_num_obstacles) {}

// This is called by Perception module at every frame to insert all
// detected obstacles.
void ObstaclesContainer::Insert(const ::google::protobuf::Message& message) {
  // Clean up the history and get the PerceptionObstacles
  curr_frame_id_mapping_.clear();
  curr_frame_movable_obstacle_ids_.clear();
  curr_frame_unmovable_obstacle_ids_.clear();
  curr_frame_considered_obstacle_ids_.clear();
  curr_frame_id_perception_obstacle_map_.clear();

  PerceptionObstacles perception_obstacles;
  perception_obstacles.CopyFrom(
      dynamic_cast<const PerceptionObstacles&>(message));

  // Get the new timestamp and update it in the class
  // - If it's more than 10sec later than the most recent one, clear the
  //   obstacle history.
  // - If it's not a valid time (earlier than history), continue.
  // - Also consider the offline_mode case.

  double timestamp = 0.0;
  if (perception_obstacles.has_header() &&
      perception_obstacles.header().has_timestamp_sec()) {
    timestamp = perception_obstacles.header().timestamp_sec();
  }
  if (std::fabs(timestamp - timestamp_) > FLAGS_replay_timestamp_gap) {
    ptr_obstacles_.Clear();
    ADEBUG << "Replay mode is enabled.";
  } else if (timestamp <= timestamp_) {
    AERROR << "Invalid timestamp curr [" << timestamp << "] v.s. prev ["
           << timestamp_ << "].";
    return;
  }

  switch (FLAGS_prediction_offline_mode) {
    case 1: {
      if (std::fabs(timestamp - timestamp_) > FLAGS_replay_timestamp_gap ||
          FeatureOutput::Size() > FLAGS_max_num_dump_feature) {
        FeatureOutput::WriteFeatureProto();
      }
      break;
    }
    case 2: {
      if (std::fabs(timestamp - timestamp_) > FLAGS_replay_timestamp_gap ||
          FeatureOutput::SizeOfDataForLearning() >
              FLAGS_max_num_dump_dataforlearn) {
        FeatureOutput::WriteDataForLearning();
      }
      break;
    }
    case 3: {
      if (std::fabs(timestamp - timestamp_) > FLAGS_replay_timestamp_gap ||
          FeatureOutput::SizeOfPredictionResult() >
              FLAGS_max_num_dump_feature) {
        FeatureOutput::WritePredictionResult();
      }
      break;
    }
    case 4: {
      if (std::fabs(timestamp - timestamp_) > FLAGS_replay_timestamp_gap ||
          FeatureOutput::SizeOfFrameEnv() > FLAGS_max_num_dump_feature) {
        FeatureOutput::WriteFrameEnv();
      }
      break;
    }
    case 5: {
      if (std::fabs(timestamp - timestamp_) > FLAGS_replay_timestamp_gap ||
          FeatureOutput::SizeOfFrameEnv() > FLAGS_max_num_dump_feature) {
        FeatureOutput::WriteDataForTuning();
      }
      break;
    }
    default: {
      // No data dump
      break;
    }
  }

  timestamp_ = timestamp;
  ADEBUG << "Current timestamp is [" << timestamp_ << "]";

  // Prediction tracking adaptation
  if (FLAGS_enable_tracking_adaptation) {
    BuildCurrentFrameIdMapping(perception_obstacles);
  }

  // Set up the ObstacleClusters:
  // 1. Initialize ObstacleClusters
  ObstacleClusters::Init();

  // 2. Insert the Obstacles one by one
  for (const PerceptionObstacle& perception_obstacle :
       perception_obstacles.perception_obstacle()) {
    ADEBUG << "Perception obstacle [" << perception_obstacle.id() << "] "
           << "was detected";
    InsertPerceptionObstacle(perception_obstacle, timestamp_);
    ADEBUG << "Perception obstacle [" << perception_obstacle.id() << "] "
           << "was inserted";
  }
  // 3. Sort the Obstacles
  ObstacleClusters::SortObstacles();
}

Obstacle* ObstaclesContainer::GetObstacle(const int id) {
  auto ptr_obstacle = ptr_obstacles_.GetSilently(id);
  if (ptr_obstacle != nullptr) {
    return ptr_obstacle->get();
  }
  return nullptr;
}

Obstacle* ObstaclesContainer::GetObstacleWithLRUUpdate(const int obstacle_id) {
  auto ptr_obstacle = ptr_obstacles_.Get(obstacle_id);
  if (ptr_obstacle != nullptr) {
    return ptr_obstacle->get();
  }
  return nullptr;
}

void ObstaclesContainer::Clear() {
  ptr_obstacles_.Clear();
  id_mapping_.Clear();
  timestamp_ = -1.0;
}

const PerceptionObstacle& ObstaclesContainer::GetPerceptionObstacle(
    const int id) {
  CHECK(curr_frame_id_perception_obstacle_map_.find(id) !=
        curr_frame_id_perception_obstacle_map_.end());
  return curr_frame_id_perception_obstacle_map_[id];
}

const std::vector<int>& ObstaclesContainer::curr_frame_movable_obstacle_ids() {
  return curr_frame_movable_obstacle_ids_;
}

const std::vector<int>&
ObstaclesContainer::curr_frame_unmovable_obstacle_ids() {
  return curr_frame_unmovable_obstacle_ids_;
}

const std::vector<int>&
ObstaclesContainer::curr_frame_considered_obstacle_ids() {
  return curr_frame_considered_obstacle_ids_;
}

void ObstaclesContainer::SetConsideredObstacleIds() {
  curr_frame_considered_obstacle_ids_.clear();
  for (const int id : curr_frame_movable_obstacle_ids_) {
    Obstacle* obstacle_ptr = GetObstacle(id);
    if (obstacle_ptr == nullptr) {
      AERROR << "Null obstacle found.";
      continue;
    }
    if (obstacle_ptr->ToIgnore()) {
      ADEBUG << "Ignore obstacle [" << obstacle_ptr->id() << "]";
      continue;
    }
    curr_frame_considered_obstacle_ids_.push_back(id);
  }
}

std::vector<int> ObstaclesContainer::curr_frame_obstacle_ids() {
  std::vector<int> curr_frame_obs_ids = curr_frame_movable_obstacle_ids_;
  curr_frame_obs_ids.insert(curr_frame_obs_ids.end(),
                            curr_frame_unmovable_obstacle_ids_.begin(),
                            curr_frame_unmovable_obstacle_ids_.end());
  return curr_frame_obs_ids;
}

void ObstaclesContainer::InsertPerceptionObstacle(
    const PerceptionObstacle& perception_obstacle, const double timestamp) {
  // Sanity checks.
  int id = PerceptionIdToPredictionId(perception_obstacle.id());
  if (id != perception_obstacle.id()) {
    ADEBUG << "Obstacle have got AdaptTracking, with perception id: "
           << perception_obstacle.id() << ", and prediction id: " << id;
  }
  curr_frame_id_perception_obstacle_map_[id] = perception_obstacle;
  if (id < FLAGS_ego_vehicle_id) {
    AERROR << "Invalid ID [" << id << "]";
    return;
  }
  if (!IsMovable(perception_obstacle)) {
    ADEBUG << "Perception obstacle [" << perception_obstacle.id()
           << "] is unmovable.";
    curr_frame_unmovable_obstacle_ids_.push_back(id);
    return;
  }

  // Insert the obstacle and also update the LRUCache.
  auto obstacle_ptr = GetObstacleWithLRUUpdate(id);
  if (obstacle_ptr != nullptr) {
    obstacle_ptr->Insert(perception_obstacle, timestamp, id);
    ADEBUG << "Refresh obstacle [" << id << "]";
  } else {
    auto ptr_obstacle = Obstacle::Create(perception_obstacle, timestamp, id);
    if (ptr_obstacle == nullptr) {
      AERROR << "Failed to insert obstacle into container";
      return;
    }
    ptr_obstacles_.Put(id, std::move(ptr_obstacle));
    ADEBUG << "Insert obstacle [" << id << "]";
  }

  if (id != FLAGS_ego_vehicle_id) {
    curr_frame_movable_obstacle_ids_.push_back(id);
  }
  SetConsideredObstacleIds();
}

void ObstaclesContainer::InsertFeatureProto(const Feature& feature) {
  if (!feature.has_id()) {
    AERROR << "Invalid feature, no ID found.";
    return;
  }
  int id = feature.id();
  auto obstacle_ptr = GetObstacleWithLRUUpdate(id);
  if (obstacle_ptr != nullptr) {
    obstacle_ptr->InsertFeature(feature);
  } else {
    auto ptr_obstacle = Obstacle::Create(feature);
    if (ptr_obstacle == nullptr) {
      AERROR << "Failed to insert obstacle into container";
      return;
    }
    ptr_obstacles_.Put(id, std::move(ptr_obstacle));
  }
}

void ObstaclesContainer::BuildCurrentFrameIdMapping(
    const PerceptionObstacles& perception_obstacles) {
  // Go through every obstacle in the current frame, after some
  // sanity checks, build current_frame_id_mapping for every obstacle

  std::unordered_set<int> seen_perception_ids;
  // Loop all precept_id and find those in obstacles_LRU
  for (const PerceptionObstacle& perception_obstacle :
       perception_obstacles.perception_obstacle()) {
    int perception_id = perception_obstacle.id();
    if (GetObstacle(perception_id) != nullptr) {
      seen_perception_ids.insert(perception_id);
    }
  }

  for (const PerceptionObstacle& perception_obstacle :
       perception_obstacles.perception_obstacle()) {
    int perception_id = perception_obstacle.id();
    curr_frame_id_mapping_[perception_id] = perception_id;
    if (seen_perception_ids.find(perception_id) != seen_perception_ids.end()) {
      // find this perception_id in LRUCache, treat it as a tracked obstacle
      continue;
    }
    std::unordered_set<int> seen_prediction_ids;
    int prediction_id = 0;
    if (id_mapping_.GetCopy(perception_id, &prediction_id)) {
      if (seen_perception_ids.find(prediction_id) ==
          seen_perception_ids.end()) {
        // find this perception_id in LRUMapping, map it to a tracked obstacle
        curr_frame_id_mapping_[perception_id] = prediction_id;
        seen_prediction_ids.insert(prediction_id);
      }
    } else {  // process adaption
      common::util::Node<int, std::unique_ptr<Obstacle>>* curr =
          ptr_obstacles_.First();
      while (curr != nullptr) {
        int obs_id = curr->key;
        curr = curr->next;
        if (obs_id < 0 ||
            seen_perception_ids.find(obs_id) != seen_perception_ids.end() ||
            seen_prediction_ids.find(obs_id) != seen_prediction_ids.end()) {
          // this obs_id has already been processed
          continue;
        }
        Obstacle* obstacle_ptr = GetObstacle(obs_id);
        if (obstacle_ptr == nullptr) {
          AERROR << "Obstacle id [" << obs_id << "] with empty obstacle_ptr.";
          break;
        }
        if (timestamp_ - obstacle_ptr->timestamp() > FLAGS_max_tracking_time) {
          ADEBUG << "Obstacle already reach time threshold.";
          break;
        }
        if (AdaptTracking(perception_obstacle, obstacle_ptr)) {
          id_mapping_.Put(perception_id, obs_id);
          curr_frame_id_mapping_[perception_id] = obs_id;
          break;
        }
      }
    }
  }
}

int ObstaclesContainer::PerceptionIdToPredictionId(const int perception_id) {
  if (curr_frame_id_mapping_.find(perception_id) ==
      curr_frame_id_mapping_.end()) {
    return perception_id;
  }
  return curr_frame_id_mapping_[perception_id];
}

void ObstaclesContainer::BuildLaneGraph() {
  // Go through every obstacle in the current frame, after some
  // sanity checks, build lane graph for non-junction cases.
  for (const int id : curr_frame_considered_obstacle_ids_) {
    Obstacle* obstacle_ptr = GetObstacle(id);
    if (obstacle_ptr == nullptr) {
      AERROR << "Null obstacle found.";
      continue;
    }
    if (FLAGS_prediction_offline_mode !=
        PredictionConstants::kDumpDataForLearning) {
      ADEBUG << "Building Lane Graph.";
      obstacle_ptr->BuildLaneGraph();
      obstacle_ptr->BuildLaneGraphFromLeftToRight();
    } else {
      ADEBUG << "Building ordered Lane Graph.";
      obstacle_ptr->BuildLaneGraphFromLeftToRight();
    }
    obstacle_ptr->SetNearbyObstacles();
  }

  Obstacle* ego_vehicle_ptr = GetObstacle(FLAGS_ego_vehicle_id);
  if (ego_vehicle_ptr == nullptr) {
    AERROR << "Ego vehicle not inserted";
    return;
  }
  ego_vehicle_ptr->BuildLaneGraph();
  ego_vehicle_ptr->SetNearbyObstacles();
}

void ObstaclesContainer::BuildJunctionFeature() {
  // Go through every obstacle in the current frame, after some
  // sanity checks, build junction features for those that are in junction.
  for (const int id : curr_frame_considered_obstacle_ids_) {
    Obstacle* obstacle_ptr = GetObstacle(id);
    if (obstacle_ptr == nullptr) {
      AERROR << "Null obstacle found.";
      continue;
    }
    const std::string& junction_id = JunctionAnalyzer::GetJunctionId();
    if (obstacle_ptr->IsInJunction(junction_id)) {
      ADEBUG << "Build junction feature for obstacle [" << obstacle_ptr->id()
             << "] in junction [" << junction_id << "]";
      obstacle_ptr->BuildJunctionFeature();
    }
  }
}

bool ObstaclesContainer::AdaptTracking(
    const PerceptionObstacle& perception_obstacle, Obstacle* obstacle_ptr) {
  if (!perception_obstacle.has_type() ||
      perception_obstacle.type() != obstacle_ptr->type()) {
    // different obstacle type, can't be same obstacle
    return false;
  }
  // test perception_obstacle position with possible obstacle position
  if (perception_obstacle.has_position() &&
      perception_obstacle.position().has_x() &&
      perception_obstacle.position().has_y()) {
    double obs_x = obstacle_ptr->latest_feature().position().x() +
                   (timestamp_ - obstacle_ptr->latest_feature().timestamp()) *
                       obstacle_ptr->latest_feature().raw_velocity().x();
    double obs_y = obstacle_ptr->latest_feature().position().y() +
                   (timestamp_ - obstacle_ptr->latest_feature().timestamp()) *
                       obstacle_ptr->latest_feature().raw_velocity().y();
    double vel_x = obstacle_ptr->latest_feature().raw_velocity().x();
    double vel_y = obstacle_ptr->latest_feature().raw_velocity().y();
    double vel = std::hypot(vel_x, vel_y);
    double dist_x = perception_obstacle.position().x() - obs_x;
    double dist_y = perception_obstacle.position().y() - obs_y;
    double dot_prod = dist_x * vel_x + dist_y * vel_y;
    double cross_prod = dist_x * vel_y - dist_y * vel_x;
    if (std::abs(dot_prod) < FLAGS_max_tracking_dist * vel &&
        std::abs(cross_prod) < FLAGS_max_tracking_dist * vel / 3) {
      return true;
    }
  }
  return false;
}

bool ObstaclesContainer::IsMovable(
    const PerceptionObstacle& perception_obstacle) {
  if (!perception_obstacle.has_type() ||
      perception_obstacle.type() == PerceptionObstacle::UNKNOWN_UNMOVABLE) {
    return false;
  }
  return true;
}

double ObstaclesContainer::timestamp() const { return timestamp_; }

}  // namespace prediction
}  // namespace apollo
