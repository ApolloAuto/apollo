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
 * @file obstacle.cc
 **/

#include "modules/planning/common/obstacle.h"

#include <algorithm>

#include "modules/common/log.h"
#include "modules/planning/common/planning_util.h"

namespace apollo {
namespace planning {

const std::string& Obstacle::Id() const { return id_; }

Obstacle::Obstacle(const std::string& id,
                   const perception::PerceptionObstacle& perception_obstacle,
                   const prediction::Trajectory& trajectory)
    : id_(id),
      trajectory_(trajectory),
      perception_obstacle_(perception_obstacle),
      perception_bounding_box_({perception_obstacle_.position().x(),
                                perception_obstacle_.position().y()},
                               perception_obstacle_.theta(),
                               perception_obstacle_.length(),
                               perception_obstacle_.width()) {}

common::TrajectoryPoint Obstacle::GetPointAtTime(
    const double relative_time) const {
  auto comp = [](const common::TrajectoryPoint p, const double relative_time) {
    return p.relative_time() < relative_time;
  };

  const auto& points = trajectory_.trajectory_point();
  auto it_lower =
      std::lower_bound(points.begin(), points.end(), relative_time, comp);

  if (it_lower == points.begin()) {
    return *points.begin();
  }
  return util::interpolate(*(it_lower - 1), *it_lower, relative_time);
}

common::math::Box2d Obstacle::GetBoundingBox(
    const common::TrajectoryPoint& point) const {
  return common::math::Box2d({point.path_point().x(), point.path_point().y()},
                             point.path_point().theta(),
                             perception_obstacle_.length(),
                             perception_obstacle_.width());
}

const common::math::Box2d& Obstacle::PerceptionBoundingBox() const {
  return perception_bounding_box_;
}

const prediction::Trajectory& Obstacle::Trajectory() const {
  return trajectory_;
}

const perception::PerceptionObstacle& Obstacle::Perception() const {
  return perception_obstacle_;
}

void Obstacle::CreateObstacles(
    const prediction::PredictionObstacles& predictions,
    std::list<std::unique_ptr<Obstacle>>* obstacles) {
  if (!obstacles) {
    AERROR << "the provided obstacles is empty";
    return;
  }
  for (const auto& prediction_obstacle : predictions.prediction_obstacle()) {
    const auto perception_id = prediction_obstacle.perception_obstacle().id();
    int trajectory_index = 0;
    for (const auto& trajectory : prediction_obstacle.trajectory()) {
      std::string obstacle_id = std::to_string(perception_id) + "_" +
                                std::to_string(trajectory_index++);
      obstacles->emplace_back(std::unique_ptr<Obstacle>(new Obstacle(
          obstacle_id, prediction_obstacle.perception_obstacle(), trajectory)));
    }
  }
}

}  // namespace planning
}  // namespace apollo
