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

#include "modules/planning/common/path_obstacle.h"

#include <limits>

#include "modules/common/log.h"

namespace apollo {
namespace planning {

const std::string& PathObstacle::Id() const { return id_; }

PathObstacle::PathObstacle(const planning::Obstacle* obstacle)
    : obstacle_(obstacle) {
  CHECK_NOTNULL(obstacle);
  id_ = obstacle_->Id();
}

bool PathObstacle::Init(const ReferenceLine* reference_line) {
  if (!InitPerceptionSLBoundary(reference_line)) {
    AERROR << "Failed to init perception sl boundary";
    return false;
  }
  return true;
}

bool PathObstacle::InitPerceptionSLBoundary(
    const ReferenceLine* reference_line) {
  double start_s(std::numeric_limits<double>::max());
  double end_s(std::numeric_limits<double>::lowest());
  double start_l(std::numeric_limits<double>::max());
  double end_l(std::numeric_limits<double>::lowest());
  std::vector<common::math::Vec2d> corners;
  obstacle_->PerceptionBoundingBox().GetAllCorners(&corners);
  for (const auto& point : corners) {
    common::SLPoint sl_point;
    if (!reference_line->get_point_in_frenet_frame(point, &sl_point)) {
      AERROR << "failed to get projection for point: " << point.DebugString()
             << " on reference line.";
      return false;
    }
    start_s = std::fmin(start_s, sl_point.s());
    end_s = std::fmax(end_s, sl_point.s());
    start_l = std::fmin(start_l, sl_point.l());
    end_l = std::fmax(end_l, sl_point.l());
  }
  perception_sl_boundary_.set_start_s(start_s);
  perception_sl_boundary_.set_end_s(end_s);
  perception_sl_boundary_.set_start_l(start_l);
  perception_sl_boundary_.set_end_l(end_l);
  return true;
}

const planning::Obstacle* PathObstacle::Obstacle() const { return obstacle_; }

void PathObstacle::AddDecision(const std::string& decider_tag,
                               const ObjectDecisionType& decision) {
  decisions_.push_back(decision);
  decider_tags_.push_back(decider_tag);
}

const std::vector<ObjectDecisionType>& PathObstacle::Decisions() const {
  return decisions_;
}

const std::string PathObstacle::DebugString() const {
  std::stringstream ss;
  for (std::size_t i = 0; i < decisions_.size(); ++i) {
    ss << "id: " << id_ << " decision: " << decisions_[i].DebugString()
       << ", made by " << decider_tags_[i];
  }
  return ss.str();
}

const SLBoundary& PathObstacle::perception_sl_boundary() const {
  return perception_sl_boundary_;
}

}  // namespace planning
}  // namespace apollo
