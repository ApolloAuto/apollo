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

#include "modules/planning/common/reference_line_info.h"

#include <functional>
#include <unordered_set>

#include "modules/planning/proto/sl_boundary.pb.h"

#include "modules/common/util/string_util.h"
#include "modules/common/util/util.h"
#include "modules/common/vehicle_state/vehicle_state.h"
#include "modules/map/hdmap/hdmap_common.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

ReferenceLineInfo::ReferenceLineInfo(const hdmap::PncMap* pnc_map,
                                     const ReferenceLine& reference_line)
    : pnc_map_(pnc_map), reference_line_(reference_line) {}

bool ReferenceLineInfo::Init() {
  const auto& box = common::VehicleState::instance()->AdcBoundingBox();
  if (!reference_line_.GetSLBoundary(box, &adc_boundary_)) {
    AERROR << "Failed to get ADC boundary from box: " << box.DebugString();
    return false;
  }
  return true;
}

const SLBoundary& ReferenceLineInfo::ADCBoundary() const {
  return adc_boundary_;
}

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
  if (!path_obstacle->Init(reference_line_)) {
    AERROR << "Failed to create perception sl boundary for obstacle "
           << obstacle->Id();
    return nullptr;
  }
  return path_obstacle;
}

std::unique_ptr<Obstacle> ReferenceLineInfo::CreateVirtualObstacle(
    const std::string& obstacle_id, const common::math::Vec2d& position,
    const double length, const double width, const double height) const {
  // create a "virtual" perception_obstacle
  perception::PerceptionObstacle perception_obstacle;
  // simulator needs a valid integer
  perception_obstacle.set_id(FLAGS_virtual_obstacle_perception_id);
  auto dest_ref_point =
      reference_line_.get_reference_point(position.x(), position.y());
  perception_obstacle.mutable_position()->set_x(position.x());
  perception_obstacle.mutable_position()->set_y(position.y());
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

const DiscretizedTrajectory& ReferenceLineInfo::trajectory() const {
  return discretized_trajectory_;
}

bool ReferenceLineInfo::IsStartFrom(
    const ReferenceLineInfo& previous_reference_line_info) const {
  if (reference_line_.reference_points().empty()) {
    return false;
  }
  auto start_point = reference_line_.reference_points().front();
  const auto& prev_reference_line =
      previous_reference_line_info.reference_line();
  common::SLPoint sl_point;
  prev_reference_line.XYToSL(start_point, &sl_point);
  return previous_reference_line_info.reference_line_.is_on_road(sl_point);
}

bool ReferenceLineInfo::IsOnLeftLane(const common::math::Vec2d& xy_point) {
  common::SLPoint sl_point;
  if (!reference_line_.XYToSL(xy_point, &sl_point)) {
    AERROR << "Failed to get sl point from : " << xy_point.DebugString();
    return false;
  }
  const double distance = 1.0;
  std::vector<hdmap::LaneInfoConstPtr> lanes;
  if (!pnc_map_->HDMap().GetLanes(common::util::MakePointENU(xy_point),
                                  distance, &lanes)) {
    AERROR << "get lanes failed from point : " << xy_point.DebugString();
    return false;
  }
  std::unordered_set<std::string> lane_ids;
  for (const auto& lane : lanes) {
    lane_ids.insert(lane->id().id());
  }
  auto ref_point = reference_line_.get_reference_point(sl_point.s());
  for (const auto& waypoint : ref_point.lane_waypoints()) {
    for (const auto& left_lane_id :
         waypoint.lane->lane().left_neighbor_forward_lane_id()) {
      if (lane_ids.count(left_lane_id.id())) {
        return true;
      }
    }
    for (const auto& left_lane_id :
         waypoint.lane->lane().left_neighbor_reverse_lane_id()) {
      if (lane_ids.count(left_lane_id.id())) {
        return true;
      }
    }
  }
  return false;
}

bool ReferenceLineInfo::IsOnRightLane(const common::math::Vec2d& xy_point) {
  common::SLPoint sl_point;
  if (!reference_line_.XYToSL(xy_point, &sl_point)) {
    AERROR << "Failed to get sl point from : " << xy_point.DebugString();
    return false;
  }
  const double distance = 1.0;
  std::vector<hdmap::LaneInfoConstPtr> lanes;
  if (pnc_map_->HDMap().GetLanes(common::util::MakePointENU(xy_point), distance,
                                 &lanes) != 0) {
    AERROR << "get lanes failed from point : " << xy_point.DebugString();
    return false;
  }
  std::unordered_set<std::string> lane_ids;
  for (const auto& lane : lanes) {
    lane_ids.insert(lane->id().id());
  }
  auto ref_point = reference_line_.get_reference_point(sl_point.s());
  for (const auto& waypoint : ref_point.lane_waypoints()) {
    for (const auto& right_lane_id :
         waypoint.lane->lane().right_neighbor_forward_lane_id()) {
      if (lane_ids.count(right_lane_id.id())) {
        return true;
      }
    }
    for (const auto& right_lane_id :
         waypoint.lane->lane().right_neighbor_reverse_lane_id()) {
      if (lane_ids.count(right_lane_id.id())) {
        return true;
      }
    }
  }
  return false;
}

const PathData& ReferenceLineInfo::path_data() const { return path_data_; }

const SpeedData& ReferenceLineInfo::speed_data() const { return speed_data_; }

PathData* ReferenceLineInfo::mutable_path_data() { return &path_data_; }

SpeedData* ReferenceLineInfo::mutable_speed_data() { return &speed_data_; }

bool ReferenceLineInfo::CombinePathAndSpeedProfile(
    const double time_resolution, const double relative_time,
    DiscretizedTrajectory* ptr_discretized_trajectory) {
  CHECK(time_resolution > 0.0);
  CHECK(ptr_discretized_trajectory != nullptr);

  for (double cur_rel_time = 0.0; cur_rel_time < speed_data_.TotalTime();
       cur_rel_time += time_resolution) {
    common::SpeedPoint speed_point;
    if (!speed_data_.EvaluateByTime(cur_rel_time, &speed_point)) {
      AERROR << "Fail to get speed point with relative time " << cur_rel_time;
      return false;
    }

    if (speed_point.s() > path_data_.discretized_path().Length()) {
      break;
    }
    common::PathPoint path_point;
    if (!path_data_.GetPathPointWithPathS(speed_point.s(), &path_point)) {
      AERROR << "Fail to get path data with s " << speed_point.s()
             << "path total length " << path_data_.discretized_path().Length();
      return false;
    }

    common::TrajectoryPoint trajectory_point;
    trajectory_point.mutable_path_point()->CopyFrom(path_point);
    trajectory_point.set_v(speed_point.v());
    trajectory_point.set_a(speed_point.a());
    trajectory_point.set_relative_time(speed_point.t() + relative_time);
    ptr_discretized_trajectory->AppendTrajectoryPoint(trajectory_point);
  }
  return true;
}

std::string ReferenceLineInfo::PathSpeedDebugString() const {
  return apollo::common::util::StrCat("path_data:", path_data_.DebugString(),
                                      "speed_data:", speed_data_.DebugString());
}

}  // namespace planning
}  // namespace apollo
