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

#include "modules/planning/planning_base/common/decision_data.h"

#include "modules/planning/planning_base/gflags/planning_gflags.h"

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

bool DecisionData::IsValidTrajectory(const prediction::Trajectory& trajectory) {
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
      all_obstacle_.emplace_back(obstacles_.back().get());
      practical_obstacle_.emplace_back(obstacles_.back().get());
      static_obstacle_.emplace_back(obstacles_.back().get());
      obstacle_map_[perception_id] = obstacles_.back().get();
      continue;
    }
    int trajectory_index = 0;
    for (const auto& trajectory : prediction_obstacle.trajectory()) {
      if (!IsValidTrajectory(trajectory)) {
        AERROR << "obj:" << perception_id;
        continue;
      }
      const std::string obstacle_id =
          absl::StrCat(perception_id, "_", trajectory_index);
      obstacles_.emplace_back(new Obstacle(
          obstacle_id, prediction_obstacle.perception_obstacle(), trajectory));
      all_obstacle_.emplace_back(obstacles_.back().get());
      practical_obstacle_.emplace_back(obstacles_.back().get());
      dynamic_obstacle_.emplace_back(obstacles_.back().get());
      obstacle_map_[obstacle_id] = obstacles_.back().get();
      ++trajectory_index;
    }
  }
}

Obstacle* DecisionData::GetObstacleById(const std::string& id) {
  std::lock_guard<std::mutex> lock(mutex_);
  return common::util::FindPtrOrNull(obstacle_map_, id);
}

std::vector<Obstacle*> DecisionData::GetObstacleByType(
    const VirtualObjectType& type) {
  std::lock_guard<std::mutex> lock(transaction_mutex_);

  std::unordered_set<std::string> ids = GetObstacleIdByType(type);
  if (ids.empty()) {
    return std::vector<Obstacle*>();
  }
  std::vector<Obstacle*> ret;
  for (const std::string& id : ids) {
    ret.emplace_back(GetObstacleById(id));
    if (ret.back() == nullptr) {
      AERROR << "Ignore. can't find obstacle by id: " << id;
      ret.pop_back();
    }
  }
  return ret;
}

std::unordered_set<std::string> DecisionData::GetObstacleIdByType(
    const VirtualObjectType& type) {
  std::lock_guard<std::mutex> lock(mutex_);
  return common::util::FindWithDefault(virtual_obstacle_id_map_, type, {});
}

const std::vector<Obstacle*>& DecisionData::GetStaticObstacle() const {
  return static_obstacle_;
}

const std::vector<Obstacle*>& DecisionData::GetDynamicObstacle() const {
  return dynamic_obstacle_;
}

const std::vector<Obstacle*>& DecisionData::GetVirtualObstacle() const {
  return virtual_obstacle_;
}

const std::vector<Obstacle*>& DecisionData::GetPracticalObstacle() const {
  return practical_obstacle_;
}

const std::vector<Obstacle*>& DecisionData::GetAllObstacle() const {
  return all_obstacle_;
}

bool DecisionData::CreateVirtualObstacle(const ReferencePoint& point,
                                         const VirtualObjectType& type,
                                         std::string* const id) {
  // should build different box by type;
  common::SLPoint sl_point;
  if (!reference_line_.XYToSL(point, &sl_point)) {
    return false;
  }
  double obstacle_s = sl_point.s();
  const double box_center_s = obstacle_s + FLAGS_virtual_stop_wall_length / 2.0;
  auto box_center = reference_line_.GetReferencePoint(box_center_s);
  double heading = reference_line_.GetReferencePoint(obstacle_s).heading();
  double lane_left_width = 0.0;
  double lane_right_width = 0.0;
  reference_line_.GetLaneWidth(obstacle_s, &lane_left_width, &lane_right_width);
  common::math::Box2d box(box_center, heading, FLAGS_virtual_stop_wall_length,
                          lane_left_width + lane_right_width);
  return CreateVirtualObstacle(box, type, id);
}

bool DecisionData::CreateVirtualObstacle(const double point_s,
                                         const VirtualObjectType& type,
                                         std::string* const id) {
  // should build different box by type;
  const double box_center_s = point_s + FLAGS_virtual_stop_wall_length / 2.0;
  auto box_center = reference_line_.GetReferencePoint(box_center_s);
  double heading = reference_line_.GetReferencePoint(point_s).heading();
  double lane_left_width = 0.0;
  double lane_right_width = 0.0;
  reference_line_.GetLaneWidth(point_s, &lane_left_width, &lane_right_width);
  common::math::Box2d box(box_center, heading, FLAGS_virtual_stop_wall_length,
                          lane_left_width + lane_right_width);
  return CreateVirtualObstacle(box, type, id);
}

bool DecisionData::CreateVirtualObstacle(
    const common::math::Box2d& obstacle_box, const VirtualObjectType& type,
    std::string* const id) {
  std::lock_guard<std::mutex> transaction_lock(transaction_mutex_);
  std::lock_guard<std::mutex> lock(mutex_);

  perception::PerceptionObstacle perception_obstacle;
  // TODO(All) please chagne is_virtual in obstacle
  perception_obstacle.set_id(virtual_obstacle_.size() + 1);
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
  *id = std::to_string(perception_obstacle.id());
  obstacles_.emplace_back(new Obstacle(*id, perception_obstacle));
  all_obstacle_.emplace_back(obstacles_.back().get());
  virtual_obstacle_.emplace_back(obstacles_.back().get());
  // would be changed if some virtual type is not static one
  static_obstacle_.emplace_back(obstacles_.back().get());

  virtual_obstacle_id_map_[type].insert(*id);
  obstacle_map_[*id] = obstacles_.back().get();
  return true;
}

}  // namespace planning
}  // namespace apollo
