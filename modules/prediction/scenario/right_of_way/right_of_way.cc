/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/prediction/scenario/right_of_way/right_of_way.h"

#include <limits>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/common/prediction_map.h"
#include "modules/prediction/container/adc_trajectory/adc_trajectory_container.h"
#include "modules/prediction/container/obstacles/obstacles_container.h"
#include "modules/prediction/container/pose/pose_container.h"

namespace apollo {
namespace prediction {

using apollo::common::adapter::AdapterConfig;
using apollo::hdmap::LaneInfo;
using apollo::perception::PerceptionObstacle;

void RightOfWay::Analyze(ContainerManager* container_manager) {
  ObstaclesContainer* obstacles_container =
      container_manager->GetContainer<ObstaclesContainer>(
          AdapterConfig::PERCEPTION_OBSTACLES);
  if (obstacles_container == nullptr) {
    AERROR << "Null obstacles container found";
    return;
  }

  ADCTrajectoryContainer* adc_trajectory_container =
      container_manager->GetContainer<ADCTrajectoryContainer>(
          AdapterConfig::PLANNING_TRAJECTORY);
  if (adc_trajectory_container == nullptr) {
    AERROR << "adc_trajectory_container is nullptr";
    return;
  }
  const std::vector<std::string>& lane_ids =
      adc_trajectory_container->GetADCLaneIDSequence();
  if (lane_ids.empty()) {
    return;
  }

  auto pose_container = container_manager->GetContainer<PoseContainer>(
      AdapterConfig::LOCALIZATION);
  if (pose_container == nullptr) {
    AERROR << "Pose container pointer is a null pointer.";
    return;
  }
  const PerceptionObstacle* pose_obstacle_ptr =
      pose_container->ToPerceptionObstacle();
  if (pose_obstacle_ptr == nullptr) {
    AERROR << "Pose obstacle pointer is a null pointer.";
    return;
  }
  double pose_x = pose_obstacle_ptr->position().x();
  double pose_y = pose_obstacle_ptr->position().y();
  double ego_vehicle_s = std::numeric_limits<double>::max();
  double ego_vehicle_l = std::numeric_limits<double>::max();
  double accumulated_s = 0.0;
  std::string ego_lane_id = "";
  // loop for lane_ids to findout ego_vehicle_s and lane_id
  for (const std::string& lane_id : lane_ids) {
    std::shared_ptr<const LaneInfo> lane_info_ptr =
        PredictionMap::LaneById(lane_id);
    if (lane_info_ptr == nullptr) {
      AERROR << "Null lane info pointer found.";
      continue;
    }
    double s = 0.0;
    double l = 0.0;
    if (PredictionMap::GetProjection({pose_x, pose_y}, lane_info_ptr, &s, &l)) {
      if (std::fabs(l) < std::fabs(ego_vehicle_l)) {
        ego_vehicle_s = accumulated_s + s;
        ego_vehicle_l = l;
        ego_lane_id = lane_id;
      }
    }
    accumulated_s += lane_info_ptr->total_length();
  }

  // loop again to assign ego_back_lane_id_set_ & ego_front_lane_id_set_
  accumulated_s = 0.0;
  std::unordered_set<std::string> ego_back_lane_id_set_;
  std::unordered_set<std::string> ego_front_lane_id_set_;
  for (const std::string& lane_id : lane_ids) {
    std::shared_ptr<const LaneInfo> lane_info_ptr =
        PredictionMap::LaneById(lane_id);
    if (lane_info_ptr == nullptr) {
      AERROR << "Null lane info pointer found.";
      continue;
    }
    accumulated_s += lane_info_ptr->total_length();
    if (accumulated_s < ego_vehicle_s) {
      ego_back_lane_id_set_.insert(lane_id);
    } else if (accumulated_s > ego_vehicle_s) {
      ego_front_lane_id_set_.insert(lane_id);
    }
  }

  // then loop through all obstacle vehicles
  for (const int id :
       obstacles_container->curr_frame_considered_obstacle_ids()) {
    Obstacle* obstacle_ptr = obstacles_container->GetObstacle(id);
    if (obstacle_ptr == nullptr) {
      continue;
    }
    Feature* latest_feature_ptr = obstacle_ptr->mutable_latest_feature();
    if (latest_feature_ptr->type() != PerceptionObstacle::VEHICLE) {
      continue;
    }
    ADEBUG << "RightOfWay for obstacle [" << latest_feature_ptr->id() << "], "
           << "with lane_sequence_size: "
           << latest_feature_ptr->lane().lane_graph().lane_sequence_size();
    for (auto& lane_sequence : *latest_feature_ptr->mutable_lane()
                                    ->mutable_lane_graph()
                                    ->mutable_lane_sequence()) {
      accumulated_s = 0.0;
      for (auto lane_segment : lane_sequence.lane_segment()) {
        accumulated_s += lane_segment.end_s() - lane_segment.start_s();
        // set lower righ_of_way for turn lanes
        if (lane_segment.lane_turn_type() == 2) {  // left_turn
          lane_sequence.set_right_of_way(-20);
        } else if (lane_segment.lane_turn_type() == 3) {  // right_turn
          lane_sequence.set_right_of_way(-10);
        }
        if (accumulated_s > 50.0) {
          break;
        }
      }
    }
  }
}

}  // namespace prediction
}  // namespace apollo
