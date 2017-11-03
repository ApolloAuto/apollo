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

/**
 * @file
 **/

#include "modules/planning/common/path_decision.h"

#include <memory>
#include <utility>

#include "modules/common/util/util.h"

namespace apollo {
namespace planning {

using IndexedPathObstacles = IndexedList<std::string, PathObstacle>;

PathObstacle *PathDecision::AddPathObstacle(const PathObstacle &path_obstacle) {
  return path_obstacles_.Add(path_obstacle.Id(), path_obstacle);
}

const IndexedPathObstacles &PathDecision::path_obstacles() const {
  return path_obstacles_;
}

PathObstacle *PathDecision::Find(const std::string &object_id) {
  return path_obstacles_.Find(object_id);
}

const PathObstacle *PathDecision::Find(const std::string &object_id) const {
  return path_obstacles_.Find(object_id);
}

void PathDecision::SetStBoundary(const std::string &id,
                                 const StBoundary &boundary) {
  auto *obstacle = path_obstacles_.Find(id);

  if (!obstacle) {
    AERROR << "Failed to find obstacle : " << id;
    return;
  } else {
    obstacle->SetStBoundary(boundary);
  }
}

bool PathDecision::AddLateralDecision(const std::string &tag,
                                      const std::string &object_id,
                                      const ObjectDecisionType &decision) {
  auto *path_obstacle = path_obstacles_.Find(object_id);
  if (!path_obstacle) {
    AERROR << "failed to find obstacle";
    return false;
  }
  path_obstacle->AddLateralDecision(tag, decision);
  return true;
}

void PathDecision::EraseStBoundaries() {
  for (const auto *path_obstacle : path_obstacles_.Items()) {
    auto *obstacle_ptr = path_obstacles_.Find(path_obstacle->Id());
    obstacle_ptr->EraseStBoundary();
  }
}

bool PathDecision::AddLongitudinalDecision(const std::string &tag,
                                           const std::string &object_id,
                                           const ObjectDecisionType &decision) {
  auto *path_obstacle = path_obstacles_.Find(object_id);
  if (!path_obstacle) {
    AERROR << "failed to find obstacle";
    return false;
  }
  path_obstacle->AddLongitudinalDecision(tag, decision);
  return true;
}

}  // namespace planning
}  // namespace apollo
