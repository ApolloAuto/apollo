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

#include <iomanip>
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

ObstaclesContainer::ObstaclesContainer()
    : ptr_obstacles_(FLAGS_max_num_obstacles) {}

ObstaclesContainer::ObstaclesContainer(const SubmoduleOutput& submodule_output)
    : ptr_obstacles_(FLAGS_max_num_obstacles) {
  for (const Obstacle& obstacle : submodule_output.curr_frame_obstacles()) {
    // Deep copy of obstacle is needed for modification
    std::unique_ptr<Obstacle> ptr_obstacle(new Obstacle(obstacle));
    ptr_obstacles_.Put(obstacle.id(), std::move(ptr_obstacle));
  }

  Obstacle ego_vehicle = submodule_output.GetEgoVehicle();
  std::unique_ptr<Obstacle> ptr_ego_vehicle(
      new Obstacle(std::move(ego_vehicle)));
  ptr_obstacles_.Put(ego_vehicle.id(), std::move(ptr_ego_vehicle));

  curr_frame_movable_obstacle_ids_ =
      submodule_output.curr_frame_movable_obstacle_ids();
  curr_frame_unmovable_obstacle_ids_ =
      submodule_output.curr_frame_unmovable_obstacle_ids();
  curr_frame_considered_obstacle_ids_ =
      submodule_output.curr_frame_considered_obstacle_ids();
}

void ObstaclesContainer::CleanUp() {
  // Clean up the history and get the PerceptionObstacles
  curr_frame_movable_obstacle_ids_.clear();
  curr_frame_unmovable_obstacle_ids_.clear();
  curr_frame_considered_obstacle_ids_.clear();
}

// This is called by Perception module at every frame to insert all
// detected obstacles.
void ObstaclesContainer::Insert(const ::google::protobuf::Message& message) {
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
      FeatureOutput::Clear();
      break;
    }
  }

  timestamp_ = timestamp;
  ADEBUG << "Current timestamp is [" << std::fixed << std::setprecision(6)
         << timestamp_ << "]";

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

  SetConsideredObstacleIds();
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
  timestamp_ = -1.0;
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
  int id = perception_obstacle.id();
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
    ADEBUG << "Current time = " << std::fixed << std::setprecision(6)
           << timestamp;
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

  if (FLAGS_prediction_offline_mode ==
          PredictionConstants::kDumpDataForLearning ||
      id != FLAGS_ego_vehicle_id) {
    curr_frame_movable_obstacle_ids_.push_back(id);
  }
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
      ADEBUG << "Building lane graph for id = " << id;
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

bool ObstaclesContainer::IsMovable(
    const PerceptionObstacle& perception_obstacle) {
  if (!perception_obstacle.has_type() ||
      perception_obstacle.type() == PerceptionObstacle::UNKNOWN_UNMOVABLE) {
    return false;
  }
  return true;
}

double ObstaclesContainer::timestamp() const { return timestamp_; }

SubmoduleOutput ObstaclesContainer::GetSubmoduleOutput(
    const size_t history_size, const absl::Time& frame_start_time) {
  SubmoduleOutput container_output;
  for (int id : curr_frame_considered_obstacle_ids_) {
    Obstacle* obstacle = GetObstacle(id);
    if (obstacle == nullptr) {
      AERROR << "Nullptr found for obstacle [" << id << "]";
      continue;
    }
    obstacle->TrimHistory(history_size);
    obstacle->ClearOldInformation();
    container_output.InsertObstacle(std::move(*obstacle));
  }

  Obstacle* ego_obstacle = GetObstacle(FLAGS_ego_vehicle_id);
  if (ego_obstacle != nullptr) {
    container_output.InsertEgoVehicle(std::move(*ego_obstacle));
  }

  container_output.set_curr_frame_movable_obstacle_ids(
      curr_frame_movable_obstacle_ids_);
  container_output.set_curr_frame_unmovable_obstacle_ids(
      curr_frame_unmovable_obstacle_ids_);
  container_output.set_curr_frame_considered_obstacle_ids(
      curr_frame_considered_obstacle_ids_);
  container_output.set_frame_start_time(frame_start_time);

  return container_output;
}

}  // namespace prediction
}  // namespace apollo
