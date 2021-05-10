/******************************************************************************
 * Copyright 2021 The Apollo Authors. All Rights Reserved.
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

#include "modules/prediction/scenario/interaction_filter/interaction_filter.h"

#include <limits>
#include <queue>
#include <unordered_map>
#include <utility>

#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/common/prediction_map.h"
#include "modules/prediction/container/adc_trajectory/adc_trajectory_container.h"
#include "modules/prediction/container/obstacles/obstacle_clusters.h"
#include "modules/prediction/container/pose/pose_container.h"
#include "modules/prediction/container/storytelling/storytelling_container.h"

namespace apollo {
namespace prediction {

using apollo::common::Point3D;
using apollo::common::adapter::AdapterConfig;
using apollo::hdmap::LaneInfo;
using apollo::hdmap::OverlapInfo;
using apollo::perception::PerceptionObstacle;
using ConstLaneInfoPtr = std::shared_ptr<const LaneInfo>;

namespace {

bool IsLaneSequenceInReferenceLine(
    const LaneSequence& lane_sequence,
    const ADCTrajectoryContainer* ego_trajectory_container) {
  for (const auto& lane_segment : lane_sequence.lane_segment()) {
    std::string lane_id = lane_segment.lane_id();
    if (ego_trajectory_container->IsLaneIdInTargetReferenceLine(lane_id)) {
      return true;
    }
  }
  return false;
}

int NearestFrontObstacleIdOnLaneSequence(const LaneSequence& lane_sequence) {
  int nearest_front_obstacle_id = std::numeric_limits<int>::min();
  double smallest_relative_s = std::numeric_limits<double>::max();
  for (const auto& nearby_obs : lane_sequence.nearby_obstacle()) {
    if (nearby_obs.s() < 0.0 ||
        nearby_obs.s() > FLAGS_interaction_search_distance_ahead) {
      continue;
    }
    if (nearby_obs.s() < smallest_relative_s) {
      smallest_relative_s = nearby_obs.s();
      nearest_front_obstacle_id = nearby_obs.id();
    }
  }
  return nearest_front_obstacle_id;
}

int NearestBackwardObstacleIdOnLaneSequence(const LaneSequence& lane_sequence) {
  int nearest_backward_obstacle_id = std::numeric_limits<int>::min();
  double smallest_relative_s = std::numeric_limits<double>::max();
  for (const auto& nearby_obs : lane_sequence.nearby_obstacle()) {
    if (nearby_obs.s() > 0.0 ||
        nearby_obs.s() < -FLAGS_interaction_search_distance_backward) {
      continue;
    }
    if (-nearby_obs.s() < smallest_relative_s) {
      smallest_relative_s = -nearby_obs.s();
      nearest_backward_obstacle_id = nearby_obs.id();
    }
  }
  return nearest_backward_obstacle_id;
}

}  // namespace

InteractionFilter::InteractionFilter(
    const std::shared_ptr<ContainerManager>& container_manager)
    : container_manager_(container_manager) {}

void InteractionFilter::AssignInteractiveTag() {
  auto obstacles_container =
      container_manager_->GetContainer<ObstaclesContainer>(
          AdapterConfig::PERCEPTION_OBSTACLES);
  if (obstacles_container == nullptr) {
    AERROR << "Obstacles container pointer is a null pointer.";
    return;
  }
  Obstacle* ego_vehicle =
      obstacles_container->GetObstacle(FLAGS_ego_vehicle_id);
  if (ego_vehicle == nullptr) {
    AERROR << "Ego vehicle not found";
    return;
  }
  if (ego_vehicle->history_size() == 0) {
    AERROR << "Ego vehicle has no history";
    return;
  }

  const auto& obstacle_ids =
    obstacles_container->curr_frame_movable_obstacle_ids();
  for (const int obstacle_id : obstacle_ids) {
    Obstacle* obstacle_ptr = obstacles_container->GetObstacle(obstacle_id);
    if (obstacle_ptr == nullptr) {
      AERROR << "Null obstacle pointer found.";
      continue;
    }
    if (obstacle_ptr->history_size() == 0) {
      AERROR << "Obstacle [" << obstacle_ptr->id() << "] has no feature.";
      continue;
    }
    Feature* latest_feature_ptr = obstacle_ptr->mutable_latest_feature();
    latest_feature_ptr->mutable_interactive_tag()->set_interactive_tag(
          ObstacleInteractiveTag::NONINTERACTION);
  }
  auto storytelling_container =
      container_manager_->GetContainer<StoryTellingContainer>(
          AdapterConfig::STORYTELLING);
  if (storytelling_container->ADCDistanceToJunction() <
      FLAGS_junction_distance_threshold) {
    AssignInteractiveTagInJunction(*ego_vehicle, obstacles_container,
                                 storytelling_container->ADCJunctionId());
  }
  AssignInteractiveTagCruiseKeepLane(*ego_vehicle, obstacles_container);
  AssignInteractiveTagCruiseChangeLane(*ego_vehicle, obstacles_container);
  AssignInteractiveTagByEgoReferenceLine(*ego_vehicle, obstacles_container);
  if (FLAGS_enable_rank_interactive_obstacles) {
    RankingInteractiveTagObstacles(*ego_vehicle, obstacles_container);
  }
}

void InteractionFilter::AssignInteractiveTagInJunction(
    const Obstacle& ego_vehicle, ObstaclesContainer* obstacles_container,
    const std::string& junction_id) {
  const auto& obstacle_ids =
      obstacles_container->curr_frame_movable_obstacle_ids();
  for (const int obstacle_id : obstacle_ids) {
    Obstacle* obstacle_ptr = obstacles_container->GetObstacle(obstacle_id);
    if (obstacle_ptr == nullptr) {
      AERROR << "Null obstacle pointer found.";
      continue;
    }
    if (obstacle_ptr->IsInJunction(junction_id)) {
      SetInteractiveIfCloseToEgo(ego_vehicle,
                             FLAGS_interaction_distance_threshold,
                             obstacle_ptr);
    }
  }
}

void InteractionFilter::AssignInteractiveTagCruiseKeepLane(
    const Obstacle& ego_vehicle, ObstaclesContainer* obstacles_container) {
  const Feature& ego_latest_feature = ego_vehicle.latest_feature();
  for (const LaneSequence& lane_sequence :
       ego_latest_feature.lane().lane_graph().lane_sequence()) {
    int nearest_front_obstacle_id =
        NearestFrontObstacleIdOnLaneSequence(lane_sequence);
    if (nearest_front_obstacle_id < 0) {
      continue;
    }
    Obstacle* obstacle_ptr =
        obstacles_container->GetObstacle(nearest_front_obstacle_id);
    if (obstacle_ptr == nullptr) {
      AERROR << "Obstacle [" << nearest_front_obstacle_id << "] Not found";
      continue;
    }
    SetInteractiveIfCloseToEgo(ego_vehicle,
                           FLAGS_interaction_distance_threshold,
                           obstacle_ptr);
  }
}

void InteractionFilter::AssignInteractiveTagCruiseChangeLane(
    const Obstacle& ego_vehicle, ObstaclesContainer* obstacles_container) {
  ADCTrajectoryContainer* ego_trajectory_container =
      container_manager_->GetContainer<ADCTrajectoryContainer>(
          AdapterConfig::PLANNING_TRAJECTORY);
  const Feature& ego_latest_feature = ego_vehicle.latest_feature();
  for (const LaneSequence& lane_sequence :
       ego_latest_feature.lane().lane_graph().lane_sequence()) {
    if (lane_sequence.vehicle_on_lane()) {
      int nearest_front_obstacle_id =
          NearestFrontObstacleIdOnLaneSequence(lane_sequence);
      if (nearest_front_obstacle_id < 0) {
        continue;
      }
      Obstacle* obstacle_ptr =
          obstacles_container->GetObstacle(nearest_front_obstacle_id);
      if (obstacle_ptr == nullptr) {
        AERROR << "Obstacle [" << nearest_front_obstacle_id << "] Not found";
        continue;
      }
      SetInteractiveIfCloseToEgo(ego_vehicle,
                             FLAGS_interaction_distance_threshold,
                             obstacle_ptr);
    } else if (IsLaneSequenceInReferenceLine(lane_sequence,
                                             ego_trajectory_container)) {
      int nearest_front_obstacle_id =
          NearestFrontObstacleIdOnLaneSequence(lane_sequence);
      int nearest_backward_obstacle_id =
          NearestBackwardObstacleIdOnLaneSequence(lane_sequence);
      if (nearest_front_obstacle_id >= 0) {
        Obstacle* front_obstacle_ptr =
            obstacles_container->GetObstacle(nearest_front_obstacle_id);
        if (front_obstacle_ptr != nullptr) {
          SetInteractiveIfCloseToEgo(ego_vehicle,
                                 FLAGS_interaction_distance_threshold,
                                 front_obstacle_ptr);
        }
      }
      if (nearest_backward_obstacle_id >= 0) {
        Obstacle* backward_obstacle_ptr =
            obstacles_container->GetObstacle(nearest_backward_obstacle_id);
        if (backward_obstacle_ptr != nullptr) {
          SetInteractiveIfCloseToEgo(ego_vehicle,
                                 FLAGS_interaction_distance_threshold,
                                 backward_obstacle_ptr);
        }
      }
    }
  }
}

void InteractionFilter::AssignInteractiveTagByEgoReferenceLine(
    const Obstacle& ego_vehicle, ObstaclesContainer* obstacles_container) {
  ADCTrajectoryContainer* adc_trajectory_container =
      container_manager_->GetContainer<ADCTrajectoryContainer>(
          AdapterConfig::PLANNING_TRAJECTORY);
  if (adc_trajectory_container == nullptr) {
    AERROR << "adc_trajectory_container is nullptr";
    return;
  }
  const std::vector<std::string>& lane_ids =
      adc_trajectory_container->GetADCTargetLaneIDSequence();
  if (lane_ids.empty()) {
    return;
  }

  const Feature& ego_feature = ego_vehicle.latest_feature();
  double ego_x = ego_feature.position().x();
  double ego_y = ego_feature.position().y();
  double ego_vehicle_s = std::numeric_limits<double>::max();
  double ego_vehicle_l = std::numeric_limits<double>::max();
  double accumulated_s = 0.0;
  // first loop for lane_ids to findout ego_vehicle_s
  for (const std::string& lane_id : lane_ids) {
    std::shared_ptr<const LaneInfo> lane_info_ptr =
        PredictionMap::LaneById(lane_id);
    if (lane_info_ptr == nullptr) {
      AERROR << "Null lane info pointer found.";
      continue;
    }
    double s = 0.0;
    double l = 0.0;
    if (PredictionMap::GetProjection({ego_x, ego_y}, lane_info_ptr, &s, &l)) {
      if (std::fabs(l) < std::fabs(ego_vehicle_l)) {
        ego_vehicle_s = accumulated_s + s;
        ego_vehicle_l = l;
        ego_lane_id_ = lane_id;
        ego_lane_s_ = s;
      }
    }
    accumulated_s += lane_info_ptr->total_length();
  }

  // insert ego_back_lane_id
  accumulated_s = 0.0;
  for (const std::string& lane_id : lane_ids) {
    if (lane_id == ego_lane_id_) {
      break;
    }
    ego_back_lane_id_set_.insert(lane_id);
  }

  std::unordered_set<std::string> visited_lanes(lane_ids.begin(),
                                                lane_ids.end());

  // then loop through lane_ids to AssignInteractive for obstacle vehicles
  for (const std::string& lane_id : lane_ids) {
    if (ego_back_lane_id_set_.find(lane_id) != ego_back_lane_id_set_.end()) {
      continue;
    }
    std::shared_ptr<const LaneInfo> lane_info_ptr =
        PredictionMap::LaneById(lane_id);
    if (lane_info_ptr == nullptr) {
      AERROR << "Null lane info pointer found.";
      continue;
    }
    accumulated_s += lane_info_ptr->total_length();
    if (lane_id != ego_lane_id_) {
      AssignInteractiveByMerge(ego_vehicle, lane_info_ptr, &visited_lanes,
                           obstacles_container);
    }
    AssignInteractiveByOverlap(ego_vehicle, lane_info_ptr, &visited_lanes,
                           obstacles_container);
    if (accumulated_s >
        FLAGS_interaction_search_distance_ahead + ego_vehicle_s) {
      break;
    }
  }
}

void InteractionFilter::RankingInteractiveTagObstacles(
    const Obstacle& ego_vehicle, ObstaclesContainer* obstacles_container) {
  const Point3D& ego_position = ego_vehicle.latest_feature().position();
  const auto& obstacle_ids =
      obstacles_container->curr_frame_movable_obstacle_ids();
  std::priority_queue<std::pair<double, Obstacle*>> interactive_obstacle_queue;
  for (const int obstacle_id : obstacle_ids) {
    Obstacle* obstacle_ptr = obstacles_container->GetObstacle(obstacle_id);
    if (obstacle_ptr == nullptr) {
      AERROR << "Obstacle [" << obstacle_id << "] Not found";
      continue;
    }
    if (!obstacle_ptr->IsInteractiveObstacle()) {
      continue;
    }
    const Point3D& obstacle_position =
        obstacle_ptr->latest_feature().position();
    double distance = std::hypot(obstacle_position.x() - ego_position.x(),
                                 obstacle_position.y() - ego_position.y());
    interactive_obstacle_queue.push({distance, obstacle_ptr});
  }
  while (static_cast<int>(interactive_obstacle_queue.size()) >
         FLAGS_interactive_obs_max_nums) {
    Obstacle* obstacle_ptr = interactive_obstacle_queue.top().second;
    obstacle_ptr->mutable_latest_feature()->
                  mutable_interactive_tag()->set_interactive_tag(
                  ObstacleInteractiveTag::INTERACTION);
    interactive_obstacle_queue.pop();
  }
}

void InteractionFilter::AssignInteractiveByMerge(
    const Obstacle& ego_vehicle, std::shared_ptr<const LaneInfo> lane_info_ptr,
    std::unordered_set<std::string>* const visited_lanes,
    ObstaclesContainer* obstacles_container) {
  SetInteractiveBackward(FLAGS_interaction_search_distance_backward_for_merge,
                     ego_vehicle, lane_info_ptr, visited_lanes,
                     obstacles_container);
}

void InteractionFilter::AssignInteractiveByOverlap(
    const Obstacle& ego_vehicle, std::shared_ptr<const LaneInfo> lane_info_ptr,
    std::unordered_set<std::string>* const visited_lanes,
    ObstaclesContainer* obstacles_container) {
  std::string lane_id = lane_info_ptr->id().id();
  const std::vector<std::shared_ptr<const OverlapInfo>> cross_lanes =
      lane_info_ptr->cross_lanes();
  for (const auto overlap_ptr : cross_lanes) {
    bool consider_overlap = true;
    for (const auto& object : overlap_ptr->overlap().object()) {
      if (object.id().id() == lane_info_ptr->id().id() &&
          object.lane_overlap_info().end_s() < ego_lane_s_) {
        consider_overlap = false;
      }
    }

    if (!consider_overlap) {
      continue;
    }

    for (const auto& object : overlap_ptr->overlap().object()) {
      const auto& object_id = object.id().id();
      if (object_id == lane_info_ptr->id().id()) {
        continue;
      }
      std::shared_ptr<const LaneInfo> overlap_lane_ptr =
          PredictionMap::LaneById(object_id);
      // ahead_s is the length in front of the overlap
      double ahead_s = overlap_lane_ptr->total_length() -
                       object.lane_overlap_info().start_s();
      SetInteractiveBackward(
          ahead_s + FLAGS_interaction_search_distance_backward_for_overlap,
          ego_vehicle, overlap_lane_ptr, visited_lanes, obstacles_container);
    }
  }
}

void InteractionFilter::SetInteractiveBackward(
    const double max_distance, const Obstacle& ego_vehicle,
    std::shared_ptr<const LaneInfo> start_lane_info_ptr,
    std::unordered_set<std::string>* const visited_lanes,
    ObstaclesContainer* obstacles_container) {
  std::string start_lane_id = start_lane_info_ptr->id().id();
  if (ego_back_lane_id_set_.find(start_lane_id) !=
      ego_back_lane_id_set_.end()) {
    return;
  }
  std::unordered_map<std::string, std::vector<LaneObstacle>> lane_obstacles =
      obstacles_container->GetClustersPtr()->GetLaneObstacles();
  std::queue<std::pair<ConstLaneInfoPtr, double>> lane_info_queue;
  lane_info_queue.emplace(start_lane_info_ptr,
                          start_lane_info_ptr->total_length());
  while (!lane_info_queue.empty()) {
    ConstLaneInfoPtr curr_lane = lane_info_queue.front().first;
    double cumu_distance = lane_info_queue.front().second;
    lane_info_queue.pop();
    const std::string& lane_id = curr_lane->id().id();
    if (visited_lanes->find(lane_id) == visited_lanes->end() &&
        lane_obstacles.find(lane_id) != lane_obstacles.end() &&
        !lane_obstacles[lane_id].empty()) {
      visited_lanes->insert(lane_id);
      // find the obstacle with largest lane_s on the lane
      int obstacle_id = lane_obstacles[lane_id].front().obstacle_id();
      double obstacle_s = lane_obstacles[lane_id].front().lane_s();
      for (const LaneObstacle& lane_obstacle : lane_obstacles[lane_id]) {
        if (lane_obstacle.lane_s() > obstacle_s) {
          obstacle_id = lane_obstacle.obstacle_id();
          obstacle_s = lane_obstacle.lane_s();
        }
      }
      Obstacle* obstacle_ptr = obstacles_container->GetObstacle(obstacle_id);
      if (obstacle_ptr == nullptr) {
        AERROR << "Obstacle [" << obstacle_id << "] Not found";
        continue;
      }
      SetInteractiveIfCloseToEgo(ego_vehicle,
                             FLAGS_interaction_distance_threshold,
                             obstacle_ptr);
      continue;
    }
    if (cumu_distance > max_distance) {
      continue;
    }
    for (const auto& pre_lane_id : curr_lane->lane().predecessor_id()) {
      if (ego_back_lane_id_set_.find(pre_lane_id.id()) !=
          ego_back_lane_id_set_.end()) {
        continue;
      }
      ConstLaneInfoPtr pre_lane_ptr = PredictionMap::LaneById(pre_lane_id.id());
      lane_info_queue.emplace(pre_lane_ptr,
                              cumu_distance + pre_lane_ptr->total_length());
    }
  }
}

void InteractionFilter::SetInteractiveIfCloseToEgo(
    const Obstacle& ego_vehicle, const double distance_threshold,
    Obstacle* obstacle_ptr) {
  const Point3D& obstacle_position = obstacle_ptr->latest_feature().position();
  const Point3D& ego_position = ego_vehicle.latest_feature().position();
  double diff_x = obstacle_position.x() - ego_position.x();
  double diff_y = obstacle_position.y() - ego_position.y();
  double distance = std::hypot(diff_x, diff_y);
  // Add interactive tag only for vehicles
  if (distance < distance_threshold &&
      obstacle_ptr->latest_feature().type() == PerceptionObstacle::VEHICLE) {
    obstacle_ptr->SetInteractiveTag();
  } else {
    obstacle_ptr->SetNonInteractiveTag();
  }
}

}  // namespace prediction
}  // namespace apollo
