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

#include "modules/planning/planning_base/common/path_decision.h"

#include "modules/common_msgs/perception_msgs/perception_obstacle.pb.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/util/util.h"

namespace apollo {
namespace planning {

Obstacle *PathDecision::AddObstacle(const Obstacle &obstacle) {
  return obstacles_.Add(obstacle.Id(), obstacle);
}

const IndexedObstacles &PathDecision::obstacles() const { return obstacles_; }

Obstacle *PathDecision::Find(const std::string &object_id) {
  return obstacles_.Find(object_id);
}

const Obstacle *PathDecision::Find(const std::string &object_id) const {
  return obstacles_.Find(object_id);
}

const perception::PerceptionObstacle *PathDecision::FindPerceptionObstacle(
    const std::string &perception_obstacle_id) const {
  for (const auto *obstacle : obstacles_.Items()) {
    if (std::to_string(obstacle->Perception().id()) == perception_obstacle_id) {
      return &(obstacle->Perception());
    }
  }

  return nullptr;
}

void PathDecision::SetSTBoundary(const std::string &id,
                                 const STBoundary &boundary) {
  auto *obstacle = obstacles_.Find(id);

  if (!obstacle) {
    AERROR << "Failed to find obstacle : " << id;
    return;
  } else {
    obstacle->set_path_st_boundary(boundary);
  }
}

bool PathDecision::AddLateralDecision(const std::string &tag,
                                      const std::string &object_id,
                                      const ObjectDecisionType &decision) {
  auto *obstacle = obstacles_.Find(object_id);
  if (!obstacle) {
    AERROR << "failed to find obstacle";
    return false;
  }
  obstacle->AddLateralDecision(tag, decision);
  return true;
}

void PathDecision::EraseStBoundaries() {
  for (const auto *obstacle : obstacles_.Items()) {
    auto *obstacle_ptr = obstacles_.Find(obstacle->Id());
    obstacle_ptr->EraseStBoundary();
  }
}

bool PathDecision::AddLongitudinalDecision(const std::string &tag,
                                           const std::string &object_id,
                                           const ObjectDecisionType &decision) {
  auto *obstacle = obstacles_.Find(object_id);
  if (!obstacle) {
    AERROR << "failed to find obstacle";
    return false;
  }
  obstacle->AddLongitudinalDecision(tag, decision);
  return true;
}

bool PathDecision::MergeWithMainStop(const ObjectStop &obj_stop,
                                     const std::string &obj_id,
                                     const ReferenceLine &reference_line,
                                     const SLBoundary &adc_sl_boundary) {
  common::PointENU stop_point = obj_stop.stop_point();
  common::SLPoint stop_line_sl;
  reference_line.XYToSL(stop_point, &stop_line_sl);

  double stop_line_s = stop_line_sl.s();
  if (stop_line_s < 0.0 || stop_line_s > reference_line.Length()) {
    AERROR << "Ignore object:" << obj_id << " fence route_s[" << stop_line_s
           << "] not in range[0, " << reference_line.Length() << "]";
    return false;
  }

  // check stop_line_s vs adc_s, ignore if it is further way than main stop
  const auto &vehicle_config = common::VehicleConfigHelper::GetConfig();
  stop_line_s = std::fmax(
      stop_line_s, adc_sl_boundary.end_s() -
                       vehicle_config.vehicle_param().front_edge_to_center());

  if (stop_line_s >= stop_reference_line_s_) {
    ADEBUG << "stop point is farther than current main stop point.";
    return false;
  }

  main_stop_.Clear();
  main_stop_.set_reason_code(obj_stop.reason_code());
  main_stop_.set_reason("stop by " + obj_id);
  main_stop_.mutable_stop_point()->set_x(obj_stop.stop_point().x());
  main_stop_.mutable_stop_point()->set_y(obj_stop.stop_point().y());
  main_stop_.set_stop_heading(obj_stop.stop_heading());
  stop_reference_line_s_ = stop_line_s;

  ADEBUG << " main stop obstacle id:" << obj_id
         << " stop_line_s:" << stop_line_s << " stop_point: ("
         << obj_stop.stop_point().x() << obj_stop.stop_point().y()
         << " ) stop_heading: " << obj_stop.stop_heading();
  return true;
}

}  // namespace planning
}  // namespace apollo
