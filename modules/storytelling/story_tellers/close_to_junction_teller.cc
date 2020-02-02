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

#include "modules/storytelling/story_tellers/close_to_junction_teller.h"

#include <string>

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/common/prediction_map.h"
#include "modules/storytelling/frame_manager.h"

namespace apollo {
namespace storytelling {
namespace {

using apollo::common::PathPoint;
using apollo::planning::ADCTrajectory;
using apollo::prediction::PredictionMap;

bool IsPointInPNCJunction(const PathPoint& point, std::string* junction_id) {
  const auto junctions = PredictionMap::GetPNCJunctions(
      {point.x(), point.y()}, FLAGS_junction_search_radius);
  if (junctions.empty() || junctions.front() == nullptr) {
    return false;
  }
  const auto& junction_info = junctions.front();
  if (junction_info != nullptr &&
      junction_info->pnc_junction().polygon().point_size() >= 3) {
    *junction_id = junction_info->id().id();
    return true;
  }
  return false;
}

bool IsPointInRegularJunction(const PathPoint& point,
                              std::string* junction_id) {
  const auto junctions = PredictionMap::GetJunctions(
      {point.x(), point.y()}, FLAGS_junction_search_radius);
  if (junctions.empty() || junctions.front() == nullptr) {
    return false;
  }
  const auto& junction_info = junctions.front();
  if (junction_info != nullptr &&
      junction_info->junction().polygon().point_size() >= 3) {
    *junction_id = junction_info->id().id();
    return true;
  }
  return false;
}

/**
 * @brief Get distance to the nearest junction within search radius.
 * @return negative if no junction ahead, 0 if in junction, or positive which is
 *         the distance to the nearest junction ahead.
 */
double DistanceToJunction(const ADCTrajectory& adc_trajectory,
                          std::string* junction_id) {
  const double s_start = adc_trajectory.trajectory_point(0).path_point().s();
  // Test for PNCJunction.
  for (const auto& point : adc_trajectory.trajectory_point()) {
    const auto& path_point = point.path_point();
    if (path_point.s() > FLAGS_adc_trajectory_search_length) {
      break;
    }
    if (IsPointInPNCJunction(path_point, junction_id)) {
      return path_point.s() - s_start;
    }
  }

  // Test for regular junction.
  for (const auto& point : adc_trajectory.trajectory_point()) {
    const auto& path_point = point.path_point();
    if (path_point.s() > FLAGS_adc_trajectory_search_length) {
      break;
    }
    if (IsPointInRegularJunction(path_point, junction_id)) {
      return path_point.s() - s_start;
    }
  }

  return -1;
}

}  // namespace

void CloseToJunctionTeller::Init() {
  auto* manager = FrameManager::Instance();
  manager->CreateOrGetReader<ADCTrajectory>(FLAGS_planning_trajectory_topic);
  CHECK(PredictionMap::Ready()) << "PredictionMap not ready";
}

void CloseToJunctionTeller::Update(Stories* stories) {
  auto* manager = FrameManager::Instance();
  static auto planning_reader = manager->CreateOrGetReader<ADCTrajectory>(
      FLAGS_planning_trajectory_topic);
  const auto trajectory = planning_reader->GetLatestObserved();
  if (trajectory == nullptr || trajectory->trajectory_point().empty()) {
    AERROR << "Planning trajectory not ready.";
    return;
  }

  std::string junction_id;
  const double distance = DistanceToJunction(*trajectory, &junction_id);
  const bool close_to_junction = distance >= 0;
  if (close_to_junction) {
    if (!stories->has_close_to_junction()) {
      AINFO << "Enter CloseToJunction story";
    }
    auto* story = stories->mutable_close_to_junction();
    story->set_distance(distance);
    story->set_junction_id(junction_id);
  } else if (stories->has_close_to_junction()) {
    AINFO << "Exit CloseToJunction story";
    stories->clear_close_to_junction();
  }
}

}  // namespace storytelling
}  // namespace apollo
