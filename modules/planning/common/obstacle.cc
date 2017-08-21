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
#include <cmath>
#include <string>

#include "modules/common/log.h"
#include "modules/common/util/string_util.h"
#include "modules/common/util/util.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/planning_util.h"

namespace apollo {
namespace planning {

const std::string& Obstacle::Id() const { return id_; }

std::int32_t Obstacle::PerceptionId() const { return perception_id_; }

Obstacle::Obstacle(const std::string& id,
                   const perception::PerceptionObstacle& perception_obstacle)
    : id_(id),
      perception_id_(perception_obstacle.id()),
      perception_obstacle_(perception_obstacle),
      perception_bounding_box_({perception_obstacle_.position().x(),
                                perception_obstacle_.position().y()},
                               perception_obstacle_.theta(),
                               perception_obstacle_.length(),
                               perception_obstacle_.width()) {
  is_static_ = IsStaticObstacle(perception_obstacle);
  is_virtual_ = IsVirtualObstacle(perception_obstacle);
}

Obstacle::Obstacle(const std::string& id,
                   const perception::PerceptionObstacle& perception_obstacle,
                   const prediction::Trajectory& trajectory)
    : Obstacle(id, perception_obstacle) {
  has_trajectory_ = true;
  trajectory_ = trajectory;
  auto& trajectory_points = *trajectory_.mutable_trajectory_point();
  double cumulative_s = 0.0;
  if (trajectory_points.size() > 0) {
    trajectory_points[0].mutable_path_point()->set_s(0.0);
  }
  for (int i = 1; i < trajectory_points.size(); ++i) {
    cumulative_s +=
        common::util::Distance2D(trajectory_points[i - 1].path_point(),
                                 trajectory_points[i].path_point());

    trajectory_points[i].mutable_path_point()->set_s(cumulative_s);
  }
}

bool Obstacle::IsStatic() const { return is_static_; }

bool Obstacle::IsVirtual() const { return is_virtual_; }

bool Obstacle::IsStaticObstacle(
    const perception::PerceptionObstacle& perception_obstacle) {
  if (perception_obstacle.type() ==
      perception::PerceptionObstacle::UNKNOWN_UNMOVABLE) {
    return true;
  }
  auto moving_speed = std::hypot(perception_obstacle.velocity().x(),
                                 perception_obstacle.velocity().y());
  return moving_speed <= FLAGS_static_speed_threshold;
}

bool Obstacle::IsVirtualObstacle(
    const perception::PerceptionObstacle &perception_obstacle) {
  return perception_obstacle.id() < 0;
}

common::TrajectoryPoint Obstacle::GetPointAtTime(
    const double relative_time) const {
  auto comp = [](const common::TrajectoryPoint p, const double time) {
    return p.relative_time() < time;
  };

  const auto& points = trajectory_.trajectory_point();
  auto it_lower =
      std::lower_bound(points.begin(), points.end(), relative_time, comp);

  if (it_lower == points.begin()) {
    return *points.begin();
  } else if (it_lower == points.end()) {
    return *points.rbegin();
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
    const auto perception_id =
        std::to_string(prediction_obstacle.perception_obstacle().id());
    if (prediction_obstacle.trajectory_size() > 0) {
      int trajectory_index = 0;
      for (const auto& trajectory : prediction_obstacle.trajectory()) {
        const std::string obstacle_id =
            apollo::common::util::StrCat(perception_id, "_", trajectory_index);
        obstacles->emplace_back(std::unique_ptr<Obstacle>(
            new Obstacle(obstacle_id, prediction_obstacle.perception_obstacle(),
                         trajectory)));
        ++trajectory_index;
      }
    } else {
      obstacles->emplace_back(std::unique_ptr<Obstacle>(new Obstacle(
          perception_id, prediction_obstacle.perception_obstacle())));
    }
  }
}

}  // namespace planning
}  // namespace apollo
