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

#include "modules/common/util/util.h"

namespace apollo {
namespace planning {

using IndexedPathObstacles = IndexedList<std::string, PathObstacle>;

PathDecision::PathDecision(const std::vector<const Obstacle *> &obstacles,
                           const ReferenceLine &reference_line)
    : reference_line_(reference_line) {
  Init(obstacles);
}

void PathDecision::Init(const std::vector<const Obstacle *> &obstacles) {
  for (const auto obstacle : obstacles) {
    auto path_obstacle = common::util::make_unique<PathObstacle>(obstacle);
    path_obstacle->Init(&reference_line_);
    path_obstacles_.Add(obstacle->Id(), std::move(path_obstacle));
  }
}

const IndexedPathObstacles &PathDecision::path_obstacles() const {
  return path_obstacles_;
}

PathObstacle *PathDecision::Find(const std::string &object_id) {
  return path_obstacles_.Find(object_id);
}

bool PathDecision::AddDecision(const std::string &tag,
                               const std::string &object_id,
                               const ObjectDecisionType &decision) {
  auto *path_obstacle = path_obstacles_.Find(object_id);
  if (!path_obstacle) {
    AERROR << "failed to find obstacle";
    return false;
  }
  path_obstacle->AddDecision(tag, decision);
  return true;
}

}  // namespace planning
}  // namespace apollo
