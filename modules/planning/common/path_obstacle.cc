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

#include <limits>
#include <unordered_map>

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/log.h"
#include "modules/common/util/util.h"
#include "modules/planning/common/path_obstacle.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/speed/st_boundary.h"

namespace apollo {
namespace planning {

using common::VehicleConfigHelper;

namespace {
const double kStBoundaryDeltaS = 0.2;
}

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

PathObstacle::PathObstacle(const Obstacle* obstacle) : obstacle_(obstacle) {
  CHECK_NOTNULL(obstacle);
  id_ = obstacle_->Id();
}

bool PathObstacle::Init(const ReferenceLine& reference_line) {
  if (!reference_line.GetSLBoundary(obstacle_->PerceptionBoundingBox(),
                                    &perception_sl_boundary_)) {
    AERROR << "Failed to get sl boundary for obstacle: " << id_;
    return false;
  }
  // BuildStBoundary(reference_line);
  return true;
}

void PathObstacle::BuildStBoundary(const ReferenceLine& reference_line) {
  if (obstacle_->IsStatic() ||
      obstacle_->Trajectory().trajectory_point().empty()) {
    std::vector<std::pair<STPoint, STPoint>> point_pairs;
    point_pairs.emplace_back(STPoint(perception_sl_boundary_.start_s(), 0.0),
                             STPoint(perception_sl_boundary_.end_s(), 0.0));
    point_pairs.emplace_back(
        STPoint(perception_sl_boundary_.start_s(), FLAGS_st_max_t),
        STPoint(perception_sl_boundary_.end_s(), FLAGS_st_max_t));
    st_boundary_ = StBoundary(point_pairs);
  } else {
    if (BuildTrajectoryStBoundary(reference_line, &st_boundary_)) {
      ADEBUG << "Found st_boundary for obstacle " << id_;
    } else {
      ADEBUG << "No st_boundary for obstacle " << id_;
    }
  }
}

bool PathObstacle::BuildTrajectoryStBoundary(
    const ReferenceLine& reference_line, StBoundary* const st_boundary) {
  const auto& object_id = obstacle_->Id();
  const auto& perception = obstacle_->Perception();
  const double object_width = perception.width();
  const double object_length = perception.length();
  const auto& trajectory_points = obstacle_->Trajectory().trajectory_point();
  if (trajectory_points.empty()) {
    AWARN << "object " << object_id << " has no trajectory points";
    return false;
  }
  const auto& adc_param =
      VehicleConfigHelper::instance()->GetConfig().vehicle_param();
  const double adc_length = adc_param.length();
  const double adc_half_length = adc_length / 2.0;
  const double adc_width = adc_param.width();
  common::math::Box2d min_box({0, 0}, 1.0, 1.0, 1.0);
  common::math::Box2d max_box({0, 0}, 1.0, 1.0, 1.0);
  const double s_step = kStBoundaryDeltaS;
  std::vector<common::math::Vec2d> polygon_points;
  for (int i = 1; i < trajectory_points.size(); ++i) {
    const auto& first_traj_point = trajectory_points[i - 1];
    const auto& second_traj_point = trajectory_points[i - 1];
    const auto& first_point = first_traj_point.path_point();
    const auto& second_point = second_traj_point.path_point();
    double total_length =
        object_length + common::util::Distance2D(first_point, second_point);
    common::math::Vec2d center((first_point.x() + second_point.x()) / 2.0,
                               (first_point.y() + second_point.y()) / 2.0);
    common::math::Box2d object_moving_box(center, first_point.theta(),
                                          total_length, object_width);
    SLBoundary object_boundary;
    if (!reference_line.GetSLBoundary(object_moving_box, &object_boundary)) {
      AERROR << "failed to calculate boundary";
      return false;
    }
    const double object_s_diff =
        object_boundary.end_s() - object_boundary.start_s();
    if (object_boundary.end_s() < 0) {
      // skip if behind adc
      continue;
    }
    const double delta_t =
        second_traj_point.relative_time() - first_traj_point.relative_time();
    for (double s = std::max(object_boundary.start_s() - adc_half_length, 0.0);
         s <
         std::min(object_boundary.end_s() + adc_half_length, FLAGS_st_max_s);
         s += s_step) {
      auto adc_trajectory_point = reference_line.GetReferencePoint(s);
      common::math::Box2d tmp_adc_box(adc_trajectory_point,
                                      adc_trajectory_point.heading(),
                                      adc_length, adc_width);
      if (tmp_adc_box.HasOverlap(object_moving_box)) {
        double mean_t = first_traj_point.relative_time();
        if (object_s_diff > common::math::kMathEpsilon) {
          mean_t = (first_traj_point.relative_time() +
                    std::fabs((s - object_boundary.start_s()) / object_s_diff) *
                        delta_t);
        }
        polygon_points.emplace_back(mean_t, s);
      }
    }
  }
  if (polygon_points.size() < 3) {
    ADEBUG << "object " << object_id << " has no st overlap";
    return false;
  }
  common::math::Polygon2d::ComputeConvexHull(polygon_points, st_boundary);
  return true;
}

const StBoundary& PathObstacle::st_boundary() const { return st_boundary_; }

bool PathObstacle::IsLateralDecision(const ObjectDecisionType& decision) {
  return decision.has_ignore() || decision.has_nudge();
}

bool PathObstacle::IsLongitudinalDecision(const ObjectDecisionType& decision) {
  return decision.has_ignore() || decision.has_stop() || decision.has_yield() ||
         decision.has_follow() || decision.has_overtake();
}

ObjectDecisionType PathObstacle::MergeLongitudinalDecision(
    const ObjectDecisionType& lhs, const ObjectDecisionType& rhs) {
  if (lhs.object_tag_case() == ObjectDecisionType::OBJECT_TAG_NOT_SET) {
    return rhs;
  }
  if (rhs.object_tag_case() == ObjectDecisionType::OBJECT_TAG_NOT_SET) {
    return lhs;
  }
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
    } else {
      DCHECK(false) << "Unknown decision";
    }
  }
}

const ObjectDecisionType& PathObstacle::LongitudinalDecision() const {
  return longitudinal_decision_;
}

const ObjectDecisionType& PathObstacle::LateralDecision() const {
  return lateral_decision_;
}

bool PathObstacle::IsIgnore() const {
  return longitudinal_decision_.has_ignore() && lateral_decision_.has_ignore();
}

ObjectDecisionType PathObstacle::MergeLateralDecision(
    const ObjectDecisionType& lhs, const ObjectDecisionType& rhs) {
  if (lhs.object_tag_case() == ObjectDecisionType::OBJECT_TAG_NOT_SET) {
    return rhs;
  }
  if (rhs.object_tag_case() == ObjectDecisionType::OBJECT_TAG_NOT_SET) {
    return lhs;
  }
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
      DCHECK(lhs.nudge().type() == rhs.nudge().type())
          << "could not merge left nudge and right nudge";
      return std::fabs(lhs.nudge().distance_l()) >
                     std::fabs(rhs.nudge().distance_l())
                 ? lhs
                 : rhs;
    }
  }
  DCHECK(false) << "Does not have rule to merge decision: "
                << lhs.ShortDebugString()
                << " and decision: " << rhs.ShortDebugString();
}

bool PathObstacle::HasLateralDecision() const {
  return lateral_decision_.object_tag_case() !=
         ObjectDecisionType::OBJECT_TAG_NOT_SET;
}

bool PathObstacle::HasLongitudinalDecision() const {
  return longitudinal_decision_.object_tag_case() !=
         ObjectDecisionType::OBJECT_TAG_NOT_SET;
}

const Obstacle* PathObstacle::obstacle() const { return obstacle_; }

void PathObstacle::AddLongitudinalDecision(const std::string& decider_tag,
                                           const ObjectDecisionType& decision) {
  DCHECK(IsLongitudinalDecision(decision))
      << "Decision: " << decision.ShortDebugString()
      << " is not a longitudinal decision";
  longitudinal_decision_ =
      MergeLongitudinalDecision(longitudinal_decision_, decision);
  ADEBUG << decider_tag << " added obstacle " << Id()
         << " a longitudinal decision: " << decision.ShortDebugString()
         << ". The merged decision is: "
         << longitudinal_decision_.ShortDebugString();
  decisions_.push_back(decision);
  decider_tags_.push_back(decider_tag);
}

void PathObstacle::AddLateralDecision(const std::string& decider_tag,
                                      const ObjectDecisionType& decision) {
  DCHECK(IsLateralDecision(decision))
      << "Decision: " << decision.ShortDebugString()
      << " is not a lateral decision";
  lateral_decision_ = MergeLateralDecision(lateral_decision_, decision);
  ADEBUG << decider_tag << " added obstacle " << Id()
         << " a lateral decision: " << decision.ShortDebugString()
         << ". The merged decision is: "
         << lateral_decision_.ShortDebugString();
  decisions_.push_back(decision);
  decider_tags_.push_back(decider_tag);
}

const std::string PathObstacle::DebugString() const {
  std::stringstream ss;
  ss << "PathObstacle id: " << id_;
  for (std::size_t i = 0; i < decisions_.size(); ++i) {
    ss << " decision: " << decisions_[i].DebugString() << ", made by "
       << decider_tags_[i];
  }
  if (lateral_decision_.object_tag_case() !=
      ObjectDecisionType::OBJECT_TAG_NOT_SET) {
    ss << "lateral decision: " << lateral_decision_.ShortDebugString();
  }
  if (longitudinal_decision_.object_tag_case() !=
      ObjectDecisionType::OBJECT_TAG_NOT_SET) {
    ss << "longitutional decision: "
       << longitudinal_decision_.ShortDebugString();
  }
  return ss.str();
}

const SLBoundary& PathObstacle::perception_sl_boundary() const {
  return perception_sl_boundary_;
}

}  // namespace planning
}  // namespace apollo
