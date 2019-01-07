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

#include <utility>

#include "modules/prediction/common/feature_output.h"
#include "modules/prediction/common/junction_analyzer.h"
#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/common/prediction_system_gflags.h"
#include "modules/prediction/container/obstacles/obstacle_clusters.h"

namespace apollo {
namespace prediction {

using apollo::perception::PerceptionObstacle;
using apollo::perception::PerceptionObstacles;


ObstaclesContainer::ObstaclesContainer()
    : obstacles_(FLAGS_max_num_obstacles) {}

// This is called by Perception module at every frame to insert all
// detected obstacles.
void ObstaclesContainer::Insert(const ::google::protobuf::Message& message) {
  // Clean up the history and get the PerceptionObstacles
  curr_frame_predictable_obstacle_ids_.clear();
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
    obstacles_.Clear();
    ADEBUG << "Replay mode is enabled.";
  } else if (timestamp <= timestamp_) {
    AERROR << "Invalid timestamp curr [" << timestamp << "] v.s. prev ["
           << timestamp_ << "].";
    return;
  }
  if (FLAGS_prediction_offline_mode) {
    if (std::fabs(timestamp - timestamp_) > FLAGS_replay_timestamp_gap ||
        FeatureOutput::Size() > FLAGS_max_num_dump_feature) {
      FeatureOutput::Write();
    }
  }
  timestamp_ = timestamp;
  ADEBUG << "Current timestamp is [" << timestamp_ << "]";

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
  // 4. Deduct the NearbyObstacles info. from the sorted Obstacles
  for (const PerceptionObstacle& perception_obstacle :
       perception_obstacles.perception_obstacle()) {
    if (IsPredictable(perception_obstacle)) {
      continue;
    }
    Obstacle* obstacle_ptr = GetObstacle(perception_obstacle.id());
    if (obstacle_ptr == nullptr) {
      continue;
    }
    obstacle_ptr->SetNearbyObstacles();
  }
}


Obstacle* ObstaclesContainer::GetObstacle(const int id) {
  return obstacles_.GetSilently(id);
}


const std::vector<int>&
ObstaclesContainer::GetCurrentFramePredictableObstacleIds() const {
  return curr_frame_predictable_obstacle_ids_;
}


void ObstaclesContainer::Clear() {
  obstacles_.Clear();
  timestamp_ = -1.0;
}


void ObstaclesContainer::InsertPerceptionObstacle(
    const PerceptionObstacle& perception_obstacle, const double timestamp) {
  // Sanity checks.
  const int id = perception_obstacle.id();
  if (id < -1) {
    AERROR << "Invalid ID [" << id << "]";
    return;
  }
  if (!IsPredictable(perception_obstacle)) {
    ADEBUG << "Perception obstacle [" << id << "] is not predictable.";
    return;
  }

  // Insert the obstacle and also update the LRUCache.
  curr_frame_predictable_obstacle_ids_.push_back(id);
  Obstacle* obstacle_ptr = obstacles_.Get(id);
  if (obstacle_ptr != nullptr) {
    obstacle_ptr->Insert(perception_obstacle, timestamp);
    ADEBUG << "Refresh obstacle [" << id << "]";
  } else {
    Obstacle obstacle;
    obstacle.Insert(perception_obstacle, timestamp);
    obstacles_.Put(id, std::move(obstacle));
    ADEBUG << "Insert obstacle [" << id << "]";
  }
}

void ObstaclesContainer::InsertFeatureProto(const Feature& feature) {
  if (!feature.has_id()) {
    AERROR << "Invalid feature, no ID found.";
    return;
  }
  int id = feature.id();
  Obstacle* obstacle_ptr = obstacles_.Get(id);
  if (obstacle_ptr != nullptr) {
    obstacle_ptr->InsertFeature(feature);
  } else {
    Obstacle obstacle;
    obstacle.InsertFeature(feature);
    obstacles_.Put(id, std::move(obstacle));
  }
}

void ObstaclesContainer::BuildLaneGraph() {
  // Go through every obstacle in the current frame, after some
  // sanity checks, build lane graph for non-junction cases.
  for (const int id : curr_frame_predictable_obstacle_ids_) {
    Obstacle* obstacle_ptr = obstacles_.GetSilently(id);
    if (obstacle_ptr == nullptr) {
      AERROR << "Null obstacle found.";
      continue;
    }
    if (obstacle_ptr->ToIgnore()) {
      ADEBUG << "Ignore obstacle [" << obstacle_ptr->id() << "]";
      continue;
    }
    obstacle_ptr->BuildLaneGraph();
  }
}


void ObstaclesContainer::BuildJunctionFeature() {
  // Go through every obstacle in the current frame, after some
  // sanit checks, build junction features for those that are in junction.
  for (const int id : curr_frame_predictable_obstacle_ids_) {
    Obstacle* obstacle_ptr = obstacles_.GetSilently(id);
    if (obstacle_ptr == nullptr) {
      AERROR << "Null obstacle found.";
      continue;
    }
    if (obstacle_ptr->ToIgnore()) {
      ADEBUG << "Ignore obstacle [" << obstacle_ptr->id() << "]";
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


bool ObstaclesContainer::IsPredictable(
    const PerceptionObstacle& perception_obstacle) {
  if (!perception_obstacle.has_type() ||
      perception_obstacle.type() == PerceptionObstacle::UNKNOWN_UNMOVABLE) {
    return false;
  }
  return true;
}

}  // namespace prediction
}  // namespace apollo
