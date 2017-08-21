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


#include <functional>

#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/proto/sl_boundary.pb.h"
#include "modules/planning/common/reference_line_info.h"

namespace apollo {
namespace planning {
uint32_t ReferenceLineInfo::s_reference_line_id_ = 0;

ReferenceLineInfo::ReferenceLineInfo(const ReferenceLine& reference_line)
    : reference_line_(reference_line) {
  id_ = std::to_string(s_reference_line_id_);
  ++s_reference_line_id_;
}

const std::string& ReferenceLineInfo::Id() const { return id_; }

PathObstacle* ReferenceLineInfo::AddObstacle(const Obstacle* obstacle) {
  auto path_obstacle = CreatePathObstacle(obstacle);
  if (!path_obstacle) {
    AERROR << "Failed to create path obstacle for " << obstacle->Id();
    return nullptr;
  }
  auto* ptr = path_obstacle.get();
  if (!path_decision_.AddPathObstacle(std::move(path_obstacle))) {
    AERROR << "Failed to add path_obstacle " << obstacle->Id();
    return nullptr;
  }
  return ptr;
}

bool ReferenceLineInfo::AddObstacles(
    const std::vector<const Obstacle*>& obstacles) {
  for (const auto* obstacle : obstacles) {
    if (!AddObstacle(obstacle)) {
      AERROR << "Failed to add obstacle " << obstacle->Id();
      return false;
    }
  }
  return true;
}

std::unique_ptr<PathObstacle> ReferenceLineInfo::CreatePathObstacle(
    const Obstacle* obstacle) {
  auto path_obstacle =
      std::unique_ptr<PathObstacle>(new PathObstacle(obstacle));
  if (!InitPerceptionSLBoundary(path_obstacle.get())) {
    AERROR << "Failed to create perception sl boundary for obstacle "
           << obstacle->Id();
    return nullptr;
  }
  return path_obstacle;
}

bool ReferenceLineInfo::InitPerceptionSLBoundary(PathObstacle* path_obstacle) {
  double start_s(std::numeric_limits<double>::max());
  double end_s(std::numeric_limits<double>::lowest());
  double start_l(std::numeric_limits<double>::max());
  double end_l(std::numeric_limits<double>::lowest());
  std::vector<common::math::Vec2d> corners;
  path_obstacle->Obstacle()->PerceptionBoundingBox().GetAllCorners(&corners);
  for (const auto& point : corners) {
    common::SLPoint sl_point;
    if (!reference_line_.get_point_in_frenet_frame(point, &sl_point)) {
      AERROR << "failed to get projection for point: " << point.DebugString()
             << " on reference line.";
      return false;
    }
    start_s = std::fmin(start_s, sl_point.s());
    end_s = std::fmax(end_s, sl_point.s());
    start_l = std::fmin(start_l, sl_point.l());
    end_l = std::fmax(end_l, sl_point.l());
  }

  SLBoundary perception_sl_boundary;
  perception_sl_boundary.set_start_s(start_s);
  perception_sl_boundary.set_end_s(end_s);
  perception_sl_boundary.set_start_l(start_l);
  perception_sl_boundary.set_end_l(end_l);
  path_obstacle->SetPerceptionSLBoundary(perception_sl_boundary);

  return true;
}

std::unique_ptr<Obstacle> ReferenceLineInfo::CreateVirtualObstacle(
    const std::string& obstacle_id, const double route_s, const double length,
    const double width, const double height) const {
  // create a "virtual" perception_obstacle
  perception::PerceptionObstacle perception_obstacle;
  // simulator needs a valid integer
  perception_obstacle.set_id(FLAGS_virtual_obstacle_perception_id);
  auto dest_ref_point = reference_line_.get_reference_point(route_s);
  perception_obstacle.mutable_position()->set_x(dest_ref_point.x());
  perception_obstacle.mutable_position()->set_y(dest_ref_point.y());
  perception_obstacle.set_theta(dest_ref_point.heading());
  perception_obstacle.mutable_velocity()->set_x(0);
  perception_obstacle.mutable_velocity()->set_y(0);
  perception_obstacle.set_length(length);
  perception_obstacle.set_width(width);
  perception_obstacle.set_height(height);
  perception_obstacle.set_type(
      perception::PerceptionObstacle::UNKNOWN_UNMOVABLE);
  perception_obstacle.set_tracking_time(1.0);

  return std::unique_ptr<Obstacle>(
      new Obstacle(obstacle_id, perception_obstacle));
}

const PlanningData& ReferenceLineInfo::planning_data() const {
  return planning_data_;
}

PlanningData* ReferenceLineInfo::mutable_planning_data() {
  return &planning_data_;
}

const DiscretizedTrajectory& ReferenceLineInfo::trajectory() const {
  return discretized_trajectory_;
}

bool ReferenceLineInfo::IsExtendedFrom(
    const ReferenceLineInfo& previous_reference_line_info) const {
  if (reference_line_.reference_points().empty()) {
    return false;
  }
  auto start_point = reference_line_.reference_points().front();
  const auto& prev_reference_line =
      previous_reference_line_info.reference_line();
  common::SLPoint sl_point;
  prev_reference_line.get_point_in_frenet_frame(start_point, &sl_point);
  return previous_reference_line_info.reference_line_.is_on_road(sl_point);
}

}  // namespace planning
}  // namespace apollo
