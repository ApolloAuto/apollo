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

DecisionData::DecisionData(const prediction::PredictionObstacles obstacles,
                        const ReferenceLine& reference_line) :
                        reference_line_(reference_line) {
}

bool DecisionData::GetObstacleById(const std::string& id,
    PathObstacle** const obstacle) {
  CHECK_NOTNULL(obstacle);
  *obstacle = nullptr;

  std::lock_guard<std::mutex> lock(mutex_);

  const auto it = obstacle_map_.find(id);
  if (it == obstacle_map_.end()) {
    return false;
  }
  *obstacle = it->second;
  return true;
}

bool DecisionData::GetObstacleByType(const VirtualObjectType& type,
                       std::vector<PathObstacle*>* const obstacles) {
  CHECK_NOTNULL(obstacles);
  obstacles->clear();

  std::lock_guard<std::mutex> lock(transaction_mutex_);

  std::vector<std::string> ids;
  if (!GetObstacleIdByType(type, &ids)) {
    return false;
  }
  for (const std::string& id : ids) {
    obstacles->emplace_back();
    CHECK(GetObstacleById(id, &(obstacles->back())));
  }
  return true;
}

bool DecisionData::GetObstacleIdByType(const VirtualObjectType& type,
                         std::vector<std::string>* const ids) {
  CHECK_NOTNULL(ids);
  ids->clear();

  std::lock_guard<std::mutex> lock(mutex_);

  const auto it = virtual_obstacle_id_map_.find(type);
  if (it == virtual_obstacle_id_map_.end()) {
    return false;
  }
  *ids = it->second;
  return true;
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

