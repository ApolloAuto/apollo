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
  std::lock_guard<std::mutex> lock(obstacle_mutex_);
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

bool PathDecision::MergeWithMainStop(const ObjectStop &obj_stop,
                                     const std::string &obj_id,
                                     const ReferenceLine &reference_line,
                                     const SLBoundary &adc_sl_boundary) {
  apollo::common::PointENU stop_point = obj_stop.stop_point();
  common::SLPoint stop_line_sl;
  reference_line.XYToSL({stop_point.x(), stop_point.y()}, &stop_line_sl);

  double stop_line_s = stop_line_sl.s();
  if (stop_line_s < 0 || stop_line_s > reference_line.Length()) {
    AERROR << "Ignore object:" << obj_id << " fence route_s[" << stop_line_s
           << "] not in range[0, " << reference_line.Length() << "]";
    return false;
  }

  // check stop_line_s vs adc_s, ignore if it is further way than main stop
  const double kStopBuff = 1.0;
  stop_line_s = std::fmax(stop_line_s, adc_sl_boundary.end_s() - kStopBuff);

  if (stop_line_s >= stop_reference_line_s_) {
    ADEBUG << "stop point is further than current main stop point.";
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
