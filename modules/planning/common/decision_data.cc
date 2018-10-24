/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/common/decision_data.h"

namespace apollo {
namespace planning {

// this sanity check will move to the very beginning of planning
bool DecisionData::IsValidTrajectoryPoint(
  const common::TrajectoryPoint& point) {
  return !((!point.has_path_point()) || std::isnan(point.path_point().x()) ||
           std::isnan(point.path_point().y()) ||
           std::isnan(point.path_point().z()) ||
           std::isnan(point.path_point().kappa()) ||
           std::isnan(point.path_point().s()) ||
           std::isnan(point.path_point().dkappa()) ||
           std::isnan(point.path_point().ddkappa()) || std::isnan(point.v()) ||
           std::isnan(point.a()) || std::isnan(point.relative_time()));
}

bool DecisionData::IsValidTrajectory(
  const prediction::Trajectory& trajectory) {
  for (const auto& point : trajectory.trajectory_point()) {
    if (!IsValidTrajectoryPoint(point)) {
      AERROR << " TrajectoryPoint: " << trajectory.ShortDebugString()
             << " is NOT valid.";
      return false;
    }
  }
  return true;
}

DecisionData::DecisionData(
    const prediction::PredictionObstacles& prediction_obstacles,
    const ReferenceLine& reference_line)
    : reference_line_(reference_line) {
  for (const auto& prediction_obstacle :
       prediction_obstacles.prediction_obstacle()) {
    const std::string perception_id =
        std::to_string(prediction_obstacle.perception_obstacle().id());
    if (prediction_obstacle.trajectory().empty()) {
      obstacles_.emplace_back(new Obstacle(
          perception_id, prediction_obstacle.perception_obstacle()));
      path_obstacles_.emplace_back(new PathObstacle(obstacles_.back().get()));
      all_obstacle_.emplace_back(path_obstacles_.back().get());
      practical_obstacle_.emplace_back(path_obstacles_.back().get());
      static_obstacle_.emplace_back(path_obstacles_.back().get());
      continue;
    }
    int trajectory_index = 0;
    for (const auto& trajectory : prediction_obstacle.trajectory()) {
      if (!IsValidTrajectory(trajectory)) {
        AERROR << "obj:" << perception_id;
        continue;
      }
      const std::string obstacle_id =
          apollo::common::util::StrCat(perception_id, "_", trajectory_index);
      obstacles_.emplace_back(new Obstacle(
          obstacle_id, prediction_obstacle.perception_obstacle(), trajectory));
      path_obstacles_.emplace_back(new PathObstacle(obstacles_.back().get()));
      all_obstacle_.emplace_back(path_obstacles_.back().get());
      practical_obstacle_.emplace_back(path_obstacles_.back().get());
      dynamic_obstacle_.emplace_back(path_obstacles_.back().get());
      ++trajectory_index;
    }
  }
}

PathObstacle* DecisionData::GetObstacleById(const std::string& id) {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = obstacle_map_.find(id);
  if (it == obstacle_map_.end()) {
    return nullptr;
  }
  return it->second;
}

std::vector<PathObstacle*> DecisionData::GetObstacleByType(
    const VirtualObjectType& type) {
  std::vector<PathObstacle*> obstacles;
  std::lock_guard<std::mutex> lock(transaction_mutex_);

  std::vector<std::string> ids = GetObstacleIdByType(type);
  if (!ids.empty()) {
    return obstacles;
  }
  for (const std::string& id : ids) {
    obstacles.emplace_back();
    obstacles.back() = GetObstacleById(id);
    CHECK_NOTNULL(obstacles.back());
  }
  return obstacles;
}

std::vector<std::string> DecisionData::GetObstacleIdByType(
    const VirtualObjectType& type) {
  std::vector<std::string> ids;
  std::lock_guard<std::mutex> lock(mutex_);

  const auto it = virtual_obstacle_id_map_.find(type);
  if (it == virtual_obstacle_id_map_.end()) {
    return ids;
  }
  return it->second;
}

const std::vector<PathObstacle*>& DecisionData::GetStaticObstacle() const {
  return static_obstacle_;
}

const std::vector<PathObstacle*>& DecisionData::GetDynamicObstacle() const {
  return dynamic_obstacle_;
}

const std::vector<PathObstacle*>& DecisionData::GetVirtualObstacle() const {
  return virtual_obstacle_;
}

const std::vector<PathObstacle*>& DecisionData::GetPracticalObstacle() const {
  return practical_obstacle_;
}

const std::vector<PathObstacle*>& DecisionData::GetAllObstacle() const {
  return all_obstacle_;
}

bool DecisionData::CreateVirtualObstacle(const ReferencePoint& point,
                                         const VirtualObjectType& type,
                                         std::string* const id) {
  std::lock_guard<std::mutex> transaction_lock(transaction_mutex_);
  std::lock_guard<std::mutex> lock(mutex_);
  return false;
}
bool DecisionData::CreateVirtualObstacle(const double point_s,
                                         const VirtualObjectType& type,
                                         std::string* const id) {
  std::lock_guard<std::mutex> transaction_lock(transaction_mutex_);
  std::lock_guard<std::mutex> lock(mutex_);
  return false;
}

}  // namespace planning
}  // namespace apollo
