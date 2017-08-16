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

#include <unordered_map>
#include <limits>

#include "modules/planning/common/path_obstacle.h"
#include "modules/common/log.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

const std::unordered_map<ObjectDecisionType::ObjectTagCase, int,
                         PathObstacle::ObjectTagCaseHash>
    PathObstacle::s_longitudinal_decision_safety_sorter_ = {
        {ObjectDecisionType::kIgnore, 0},
        {ObjectDecisionType::kOvertake, 100},
        {ObjectDecisionType::kFollow, 300},
        {ObjectDecisionType::kYield, 400},
        {ObjectDecisionType::kStop, 500}};

const std::unordered_map<ObjectDecisionType::ObjectTagCase, int,
                         PathObstacle::ObjectTagCaseHash>
    PathObstacle::s_lateral_decision_safety_sorter_ = {
        {ObjectDecisionType::kIgnore, 0}, {ObjectDecisionType::kNudge, 100}};

const std::string& PathObstacle::Id() const { return id_; }

PathObstacle::PathObstacle() {
  lateral_decision_.mutable_ignore();
  longitudinal_decision_.mutable_ignore();
}

PathObstacle::PathObstacle(const planning::Obstacle* obstacle)
    : obstacle_(obstacle) {
  CHECK_NOTNULL(obstacle);
  id_ = obstacle_->Id();
  lateral_decision_.mutable_ignore();
  longitudinal_decision_.mutable_ignore();
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

bool PathObstacle::IsLateralDecision(const ObjectDecisionType& decision) {
  return decision.has_ignore() || decision.has_nudge();
}

bool PathObstacle::IsLongitudinalDecision(const ObjectDecisionType& decision) {
  return decision.has_ignore() || decision.has_stop() || decision.has_yield() ||
         decision.has_follow() || decision.has_overtake();
}

ObjectDecisionType PathObstacle::MergeLongitudinalDecision(
    const ObjectDecisionType& lhs, const ObjectDecisionType& rhs) {
  auto lhs_iter =
      s_longitudinal_decision_safety_sorter_.find(lhs.object_tag_case());
  DCHECK(lhs_iter != s_longitudinal_decision_safety_sorter_.end())
      << "decision : " << lhs.ShortDebugString()
      << " not found in safety sorter";
  auto rhs_iter =
      s_longitudinal_decision_safety_sorter_.find(rhs.object_tag_case());
  DCHECK(rhs_iter != s_longitudinal_decision_safety_sorter_.end())
      << "decision : " << rhs.ShortDebugString()
      << " not found in safety sorter";
  if (lhs_iter->second < rhs_iter->second) {
    return rhs;
  } else if (lhs_iter->second > rhs_iter->second) {
    return lhs;
  } else {
    if (lhs.has_ignore()) {
      return rhs;
    } else if (lhs.has_stop()) {
      return lhs.stop().distance_s() < rhs.stop().distance_s() ? lhs : rhs;
    } else if (lhs.has_yield()) {
      return lhs.yield().distance_s() < rhs.yield().distance_s() ? lhs : rhs;
    } else if (lhs.has_follow()) {
      return lhs.follow().distance_s() < rhs.follow().distance_s() ? lhs : rhs;
    } else if (lhs.has_overtake()) {
      return lhs.overtake().distance_s() > rhs.overtake().distance_s() ? lhs
                                                                       : rhs;
    }
  }
  DCHECK(false) << "Does not have rule to merge decision: "
                << lhs.ShortDebugString()
                << " and decision: " << rhs.ShortDebugString();
}

const ObjectDecisionType& PathObstacle::LongitudinalDecision() const {
  return longitudinal_decision_;
}

const ObjectDecisionType& PathObstacle::LateralDecision() const {
  return lateral_decision_;
}

ObjectDecisionType PathObstacle::MergeLateralDecision(
    const ObjectDecisionType& lhs, const ObjectDecisionType& rhs) {
  auto lhs_iter = s_lateral_decision_safety_sorter_.find(lhs.object_tag_case());
  DCHECK(lhs_iter != s_lateral_decision_safety_sorter_.end())
      << "decision : " << lhs.ShortDebugString()
      << " not found in safety sorter";
  auto rhs_iter = s_lateral_decision_safety_sorter_.find(rhs.object_tag_case());
  DCHECK(rhs_iter != s_lateral_decision_safety_sorter_.end())
      << "decision : " << rhs.ShortDebugString()
      << " not found in safety sorter";
  if (lhs_iter->second < rhs_iter->second) {
    return rhs;
  } else if (lhs_iter->second > rhs_iter->second) {
    return lhs;
  } else {
    if (lhs.has_ignore()) {
      return rhs;
    } else if (lhs.has_nudge()) {
      if (lhs.nudge().type() == rhs.nudge().type()) {
        return std::fabs(lhs.nudge().distance_l()) >
                       std::fabs(rhs.nudge().distance_l())
                   ? lhs
                   : rhs;
      } else {
        ObjectDecisionType stop_decision;
        stop_decision.mutable_stop()->set_distance_s(
            FLAGS_static_decision_stop_buffer);
        return stop_decision;
      }
    }
  }
  DCHECK(false) << "Does not have rule to merge decision: "
                << lhs.ShortDebugString()
                << " and decision: " << rhs.ShortDebugString();
}

bool PathObstacle::HasLateralDecision() const { return has_lateral_decision_; }

bool PathObstacle::HasLongitudinalDecision() const {
  return has_longitudinal_decision_;
}

const planning::Obstacle* PathObstacle::Obstacle() const { return obstacle_; }

void PathObstacle::AddDecision(const std::string& decider_tag,
                               const ObjectDecisionType& decision) {
  if (decision.has_ignore()) {
    if (!has_lateral_decision_) {
      lateral_decision_ = decision;
      has_lateral_decision_ = true;
    }
    if (!has_longitudinal_decision_) {
      longitudinal_decision_ = decision;
      has_longitudinal_decision_ = true;
    }
    return;
  }

  if (IsLateralDecision(decision)) {
    const auto merged_decision =
        MergeLateralDecision(lateral_decision_, decision);
    if (IsLongitudinalDecision(merged_decision)) {
      longitudinal_decision_ =
          MergeLongitudinalDecision(longitudinal_decision_, merged_decision);
      has_longitudinal_decision_ = true;
      lateral_decision_.mutable_ignore();
      has_lateral_decision_ = true;
    } else {
      lateral_decision_ = merged_decision;
      has_lateral_decision_ = true;
    }
  } else if (IsLongitudinalDecision(decision)) {
    longitudinal_decision_ =
        MergeLongitudinalDecision(longitudinal_decision_, decision);
    has_longitudinal_decision_ = true;
  } else {
    DCHECK(false) << "Unkown decision type: " << decision.DebugString();
  }
  decisions_.push_back(decision);
  decider_tags_.push_back(decider_tag);
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
