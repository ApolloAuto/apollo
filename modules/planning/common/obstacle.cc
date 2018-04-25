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

#include "modules/common/configs/config_gflags.h"
#include "modules/common/log.h"
#include "modules/common/math/linear_interpolation.h"
#include "modules/common/util/string_util.h"
#include "modules/common/util/util.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::perception::PerceptionObstacle;

const std::string& Obstacle::Id() const { return id_; }

std::int32_t Obstacle::PerceptionId() const { return perception_id_; }

Obstacle::Obstacle(const std::string& id,
                   const PerceptionObstacle& perception_obstacle)
    : id_(id),
      perception_id_(perception_obstacle.id()),
      perception_obstacle_(perception_obstacle),
      perception_bounding_box_({perception_obstacle_.position().x(),
                                perception_obstacle_.position().y()},
                               perception_obstacle_.theta(),
                               perception_obstacle_.length(),
                               perception_obstacle_.width()) {
  std::vector<common::math::Vec2d> polygon_points;
  if (FLAGS_use_navigation_mode ||
      perception_obstacle.polygon_point_size() <= 2) {
    perception_bounding_box_.GetAllCorners(&polygon_points);
  } else {
    CHECK(perception_obstacle.polygon_point_size() > 2)
        << "object " << id << "has less than 3 polygon points";
    for (const auto& point : perception_obstacle.polygon_point()) {
      polygon_points.emplace_back(point.x(), point.y());
    }
  }
  CHECK(common::math::Polygon2d::ComputeConvexHull(polygon_points,
                                                   &perception_polygon_))
      << "object[" << id << "] polygon is not a valid convex hull";

  is_static_ = IsStaticObstacle(perception_obstacle);
  is_virtual_ = IsVirtualObstacle(perception_obstacle);
  speed_ = std::hypot(perception_obstacle.velocity().x(),
                      perception_obstacle.velocity().y());
}

Obstacle::Obstacle(const std::string& id,
                   const PerceptionObstacle& perception_obstacle,
                   const prediction::Trajectory& trajectory)
    : Obstacle(id, perception_obstacle) {
  trajectory_ = trajectory;
  auto& trajectory_points = *trajectory_.mutable_trajectory_point();
  double cumulative_s = 0.0;
  if (trajectory_points.size() > 0) {
    trajectory_points[0].mutable_path_point()->set_s(0.0);
  }
  for (int i = 1; i < trajectory_points.size(); ++i) {
    const auto& prev = trajectory_points[i - 1];
    const auto& cur = trajectory_points[i];
    if (prev.relative_time() >= cur.relative_time()) {
      AERROR << "prediction time is not increasing."
             << "current point: " << cur.ShortDebugString()
             << "previous point: " << prev.ShortDebugString();
    }
    cumulative_s +=
        common::util::DistanceXY(prev.path_point(), cur.path_point());
    trajectory_points[i].mutable_path_point()->set_s(cumulative_s);
  }
  speed_ = std::hypot(perception_obstacle.velocity().x(),
                      perception_obstacle.velocity().y());
}

double Obstacle::Speed() const { return speed_; }

bool Obstacle::IsStatic() const { return is_static_; }

bool Obstacle::IsVirtual() const { return is_virtual_; }

bool Obstacle::HasTrajectory() const {
  return trajectory_.trajectory_point_size() > 0;
}

common::TrajectoryPoint* Obstacle::AddTrajectoryPoint() {
  return trajectory_.add_trajectory_point();
}

bool Obstacle::IsStaticObstacle(const PerceptionObstacle& perception_obstacle) {
  if (perception_obstacle.type() == PerceptionObstacle::UNKNOWN_UNMOVABLE) {
    return true;
  }
  auto moving_speed = std::hypot(perception_obstacle.velocity().x(),
                                 perception_obstacle.velocity().y());
  return moving_speed <= FLAGS_static_obstacle_speed_threshold;
}

bool Obstacle::IsVirtualObstacle(
    const PerceptionObstacle& perception_obstacle) {
  return perception_obstacle.id() < 0;
}

common::TrajectoryPoint Obstacle::GetPointAtTime(
    const double relative_time) const {
  const auto& points = trajectory_.trajectory_point();
  if (points.size() < 2) {
    common::TrajectoryPoint point;
    point.mutable_path_point()->set_x(perception_obstacle_.position().x());
    point.mutable_path_point()->set_y(perception_obstacle_.position().y());
    point.mutable_path_point()->set_z(perception_obstacle_.position().z());
    point.mutable_path_point()->set_theta(perception_obstacle_.theta());
    point.mutable_path_point()->set_s(0.0);
    point.mutable_path_point()->set_kappa(0.0);
    point.mutable_path_point()->set_dkappa(0.0);
    point.mutable_path_point()->set_ddkappa(0.0);
    point.set_v(0.0);
    point.set_a(0.0);
    point.set_relative_time(0.0);
    return point;
  } else {
    auto comp = [](const common::TrajectoryPoint p, const double time) {
      return p.relative_time() < time;
    };

    auto it_lower =
        std::lower_bound(points.begin(), points.end(), relative_time, comp);

    if (it_lower == points.begin()) {
      return *points.begin();
    } else if (it_lower == points.end()) {
      return *points.rbegin();
    }
    return common::math::InterpolateUsingLinearApproximation(
        *(it_lower - 1), *it_lower, relative_time);
  }
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

const PerceptionObstacle& Obstacle::Perception() const {
  return perception_obstacle_;
}

const common::math::Polygon2d& Obstacle::PerceptionPolygon() const {
  return perception_polygon_;
}

std::list<std::unique_ptr<Obstacle>> Obstacle::CreateObstacles(
    const prediction::PredictionObstacles& predictions) {
  std::list<std::unique_ptr<Obstacle>> obstacles;
  for (const auto& prediction_obstacle : predictions.prediction_obstacle()) {
    const auto perception_id =
        std::to_string(prediction_obstacle.perception_obstacle().id());
    if (prediction_obstacle.trajectory().empty()) {
      obstacles.emplace_back(new Obstacle(
          perception_id, prediction_obstacle.perception_obstacle()));
      continue;
    }

    int trajectory_index = 0;
    for (const auto& trajectory : prediction_obstacle.trajectory()) {
      bool is_valid_trajectory = true;
      for (const auto& point : trajectory.trajectory_point()) {
        if (!IsValidTrajectoryPoint(point)) {
          AERROR << "obj:" << perception_id
                 << " TrajectoryPoint: " << trajectory.ShortDebugString()
                 << " is NOT valid.";
          is_valid_trajectory = false;
          break;
        }
      }
      if (!is_valid_trajectory) {
        continue;
      }

      const std::string obstacle_id =
          apollo::common::util::StrCat(perception_id, "_", trajectory_index);
      obstacles.emplace_back(new Obstacle(
          obstacle_id, prediction_obstacle.perception_obstacle(), trajectory));
      ++trajectory_index;
    }
  }
  return obstacles;
}

bool Obstacle::IsValidTrajectoryPoint(const common::TrajectoryPoint& point) {
  return !((!point.has_path_point()) || std::isnan(point.path_point().x()) ||
           std::isnan(point.path_point().y()) ||
           std::isnan(point.path_point().z()) ||
           std::isnan(point.path_point().kappa()) ||
           std::isnan(point.path_point().s()) ||
           std::isnan(point.path_point().dkappa()) ||
           std::isnan(point.path_point().ddkappa()) || std::isnan(point.v()) ||
           std::isnan(point.a()) || std::isnan(point.relative_time()));
}

std::unique_ptr<Obstacle> Obstacle::CreateStaticVirtualObstacles(
    const std::string& id, const common::math::Box2d& obstacle_box) {
  // create a "virtual" perception_obstacle
  perception::PerceptionObstacle perception_obstacle;
  // simulator needs a valid integer
  int32_t negative_id = std::hash<std::string>{}(id);
  // set the first bit to 1 so negative_id became negative number
  negative_id |= (0x1 << 31);
  perception_obstacle.set_id(negative_id);
  perception_obstacle.mutable_position()->set_x(obstacle_box.center().x());
  perception_obstacle.mutable_position()->set_y(obstacle_box.center().y());
  perception_obstacle.set_theta(obstacle_box.heading());
  perception_obstacle.mutable_velocity()->set_x(0);
  perception_obstacle.mutable_velocity()->set_y(0);
  perception_obstacle.set_length(obstacle_box.length());
  perception_obstacle.set_width(obstacle_box.width());
  perception_obstacle.set_height(FLAGS_virtual_stop_wall_height);
  perception_obstacle.set_type(
      perception::PerceptionObstacle::UNKNOWN_UNMOVABLE);
  perception_obstacle.set_tracking_time(1.0);

  std::vector<common::math::Vec2d> corner_points;
  obstacle_box.GetAllCorners(&corner_points);
  for (const auto& corner_point : corner_points) {
    auto* point = perception_obstacle.add_polygon_point();
    point->set_x(corner_point.x());
    point->set_y(corner_point.y());
  }
  auto* obstacle = new Obstacle(id, perception_obstacle);
  obstacle->is_virtual_ = true;
  return std::unique_ptr<Obstacle>(obstacle);
}

}  // namespace planning
}  // namespace apollo
