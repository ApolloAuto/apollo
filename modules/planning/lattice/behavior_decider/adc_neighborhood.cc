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

#include "modules/planning/lattice/behavior_decider/adc_neighborhood.h"

#include <utility>
#include <vector>
#include <cmath>

#include "modules/planning/proto/sl_boundary.pb.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/lattice/util/lattice_params.h"
#include "modules/planning/lattice/util/reference_line_matcher.h"
#include "modules/planning/lattice/util/lattice_util.h"

namespace apollo {
namespace planning {

using apollo::common::PathPoint;
using apollo::common::TrajectoryPoint;
using apollo::perception::PerceptionObstacle;

ADCNeighborhood::ADCNeighborhood(
    const Frame* frame,
    const TrajectoryPoint& planning_init_point,
    const ReferenceLine& reference_line) {
  InitNeighborhood(frame, planning_init_point, reference_line);
}

void ADCNeighborhood::InitNeighborhood(
    const Frame* frame,
    const TrajectoryPoint& planning_init_point,
    const ReferenceLine& reference_line) {
  SetupADC(frame, planning_init_point, reference_line);
  SetupObstacles(frame, reference_line);
}

void ADCNeighborhood::SetupADC(
    const Frame* frame,
    const TrajectoryPoint& planning_init_point,
    const ReferenceLine& reference_line) {
  auto discretized_reference_line =
      ToDiscretizedReferenceLine(
          reference_line.reference_points());
  PathPoint matched_point = ReferenceLineMatcher::MatchToReferenceLine(
      discretized_reference_line, planning_init_point.path_point().x(),
      planning_init_point.path_point().y());
  std::array<double, 3> init_s;
  std::array<double, 3> init_d;
  ComputeInitFrenetState(matched_point, planning_init_point,
      &init_s, &init_d);
  init_s_ = init_s;
  init_d_ = init_d;
}

void ADCNeighborhood::SetupObstacles(
    const Frame* frame,
    const ReferenceLine& reference_line) {
  const std::vector<const Obstacle*>& obstacles = frame->obstacles();
  std::vector<PathPoint> discretized_ref_points =
      ToDiscretizedReferenceLine(reference_line.reference_points());
  for (const Obstacle* obstacle : obstacles) {
    // TODO(zhangyajia): each obstacle has only one predicted trajectory?
    if (obstacle->Trajectory().trajectory_point_size() == 0) {
      continue;
    }
    double relative_time = 0.0;
    bool entered = false;
    while (relative_time < planned_trajectory_time) {
      common::TrajectoryPoint point = obstacle->GetPointAtTime(relative_time);
      common::math::Box2d box = obstacle->GetBoundingBox(point);
      bool overlap = reference_line.HasOverlap(box);
      if (overlap) {
        SLBoundary sl_boundary;
        reference_line.GetSLBoundary(box, &sl_boundary);
        
        // TODO(all) confirm the logic to determine whether an obstacle
        // is forward or backward.
        if (sl_boundary.end_s() < init_s_[0]) {
          backward_obstacle_id_set_.insert(obstacle->Id());
        } else {
          forward_obstacle_id_set_.insert(obstacle->Id());
        }

        // TODO(kechxu) move const 2.0 in some place
        if (std::abs(sl_boundary.start_l()) > lateral_enter_lane_thred &&
            std::abs(sl_boundary.end_l()) > lateral_enter_lane_thred) {
          if (!entered) {
            continue;
          } else {
            break;
          }
        }
        double v = SpeedOnReferenceLine(discretized_ref_points,
              obstacle, sl_boundary);
        critical_conditions_[obstacle->Id()].set_obstacle_id(
              obstacle->Id());
        if (!entered) {
          SetCriticalPoint(relative_time, sl_boundary.start_s(), v,
              critical_conditions_[obstacle->Id()].mutable_bottom_left());
          SetCriticalPoint(relative_time, sl_boundary.end_s(), v,
              critical_conditions_[obstacle->Id()].mutable_upper_left());
          entered = !entered;
        }
        SetCriticalPoint(relative_time, sl_boundary.start_s(), v,
            critical_conditions_[obstacle->Id()].mutable_bottom_right());
        SetCriticalPoint(relative_time, sl_boundary.end_s(), v,
            critical_conditions_[obstacle->Id()].mutable_upper_right());
      }
      relative_time += trajectory_time_resolution;
    }
  }
}

void ADCNeighborhood::SetCriticalPoint(
    const double t, const double s, const double v,
    CriticalPoint* critical_point) {
  critical_point->set_t(t);
  critical_point->set_s(s);
  critical_point->set_v(v);
}

double ADCNeighborhood::SpeedOnReferenceLine(
    const std::vector<apollo::common::PathPoint>& discretized_ref_points,
    const Obstacle* obstacle,
    const SLBoundary& sl_boundary) {
  PathPoint obstacle_point_on_ref_line =
      ReferenceLineMatcher::MatchToReferenceLine(
          discretized_ref_points, sl_boundary.start_s());
  const PerceptionObstacle& perception_obstacle =
      obstacle->Perception();
  double ref_theta = obstacle_point_on_ref_line.theta();
  auto velocity = perception_obstacle.velocity();
  double v = std::cos(ref_theta) * velocity.x() +
      std::sin(ref_theta) * velocity.y();
  return v;
}

void ADCNeighborhood::GetCriticalConditions(
    std::vector<CriticalCondition>* critical_conditions) {
  critical_conditions->clear();
  for (const auto& condition : critical_conditions_) {
    critical_conditions->push_back(condition.second);
  }
}

bool ADCNeighborhood::GetCriticalCondition(
    const std::string& obstacle_id,
    CriticalCondition* critical_condition) {
  if (forward_obstacle_id_set_.find(obstacle_id) ==
      forward_obstacle_id_set_.end() ||
      backward_obstacle_id_set_.find(obstacle_id) ==
      backward_obstacle_id_set_.end()) {
    return false;
  }
  *critical_condition = critical_conditions_[obstacle_id];
  return true;
}

bool ADCNeighborhood::ForwardNearestObstacle(
    std::array<double, 3>* forward_nearest_obstacle_state,
    double* enter_time) {
  bool found = false;
  for (const auto& obstacle_state : forward_neighborhood_) {
    double obstacle_s = obstacle_state[1];
    // TODO(all) consider the length of adc,
    // Maybe change init_s_[0] to init_s_[0] - half_adc_length
    if (!found) {
      found = true;
      *enter_time = obstacle_state[0];
      (*forward_nearest_obstacle_state)[0] = obstacle_state[1];
      (*forward_nearest_obstacle_state)[1] = obstacle_state[3];
      (*forward_nearest_obstacle_state)[2] = obstacle_state[4];
    } else if (obstacle_s < (*forward_nearest_obstacle_state)[0]) {
      *enter_time = obstacle_state[0];
      (*forward_nearest_obstacle_state)[0] = obstacle_state[1];
      (*forward_nearest_obstacle_state)[1] = obstacle_state[3];
      (*forward_nearest_obstacle_state)[2] = obstacle_state[4];
    }
  }
  return found;
}

bool ADCNeighborhood::BackwardNearestObstacle(
    std::array<double, 3>* backward_nearest_obstacle_state,
    double* enter_time) {
  bool found = false;
  for (const auto& obstacle_state : backward_neighborhood_) {
    double obstacle_s = obstacle_state[2];
    // TODO(all) consider the length of adc,
    // Maybe change init_s_[0] to init_s_[0] + half_adc_length
    if (obstacle_s > init_s_[0]) {
      continue;
    }
    if (!found) {
      found = true;
      *enter_time = obstacle_state[0];
      (*backward_nearest_obstacle_state)[0] = obstacle_state[2];
      (*backward_nearest_obstacle_state)[1] = obstacle_state[3];
      (*backward_nearest_obstacle_state)[2] = obstacle_state[4];
    } else if (obstacle_s > (*backward_nearest_obstacle_state)[0]) {
      *enter_time = obstacle_state[0];
      (*backward_nearest_obstacle_state)[0] = obstacle_state[2];
      (*backward_nearest_obstacle_state)[1] = obstacle_state[3];
      (*backward_nearest_obstacle_state)[2] = obstacle_state[4];
    }
  }
  return found;
}

bool ADCNeighborhood::IsForward(const Obstacle* obstacle) const {
  std::string obstacle_id = obstacle->Id();
  return forward_obstacle_id_set_.find(obstacle_id) !=
         forward_obstacle_id_set_.end();
}

bool ADCNeighborhood::IsBackward(const Obstacle* obstacle) const {
  std::string obstacle_id = obstacle->Id();
  return backward_obstacle_id_set_.find(obstacle_id) !=
         backward_obstacle_id_set_.end();
}

bool ADCNeighborhood::IsInNeighborhood(const Obstacle* obstacle) const {
  return IsForward(obstacle) || IsBackward(obstacle);
}

}  // namespace planning
}  // namespace apollo
