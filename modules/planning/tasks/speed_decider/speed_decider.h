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

#ifndef MODULES_PLANNING_TASKS_SPEED_DECIDER_SPEED_DECIDER_H_
#define MODULES_PLANNING_TASKS_SPEED_DECIDER_SPEED_DECIDER_H_

#include <string>

#include "modules/planning/proto/dp_st_speed_config.pb.h"
#include "modules/planning/proto/st_boundary_config.pb.h"

#include "modules/planning/tasks/task.h"
namespace apollo {
namespace planning {

class SpeedDecider : public Task {
 public:
  SpeedDecider();
  ~SpeedDecider() = default;

  bool Init(const PlanningConfig& config) override;

  apollo::common::Status Execute(
      Frame* frame, ReferenceLineInfo* reference_line_info) override;

 private:
  enum StPosition {
    ABOVE = 1,
    BELOW = 2,
    CROSS = 3,
  };

  StPosition GetStPosition(const SpeedData& speed_profile,
                           const StBoundary& st_boundary) const;
  /**
   * @brief check if the ADC should follow an obstacle by examing the
   *StBoundary of the obstacle.
   * @param boundary The boundary of the obstacle.
   * @return true if the ADC believe it should follow the obstacle, and
   *         false otherwise.
   **/
  bool CheckIsFollowByT(const StBoundary& boundary) const;

  bool CreateStopDecision(const PathObstacle& path_obstacle,
                          ObjectDecisionType* const stop_decision,
                          double stop_distance) const;

  /**
   * @brief create follow decision based on the boundary
   **/
  bool CreateFollowDecision(const PathObstacle& path_obstacle,
                            ObjectDecisionType* const follow_decision) const;

  /**
   * @brief create yield decision based on the boundary
   **/
  bool CreateYieldDecision(const PathObstacle& path_obstacle,
                           ObjectDecisionType* const yield_decision) const;

  /**
   * @brief create overtake decision based on the boundary
   **/
  bool CreateOvertakeDecision(
      const PathObstacle& path_obstacle,
      ObjectDecisionType* const overtake_decision) const;

  apollo::common::Status MakeObjectDecision(
      const SpeedData& speed_profile, PathDecision* const path_decision) const;

  void AppendIgnoreDecision(PathObstacle* path_obstacle) const;

  /**
   * @brief "too close" is determined by whether ego vehicle will hit the front
   * obstacle if the obstacle drive at current speed and ego vehicle use some
   * reasonable deceleration
   **/
  bool IsFollowTooClose(const PathObstacle& path_obstacle) const;

 private:
  DpStSpeedConfig dp_st_speed_config_;
  StBoundaryConfig st_boundary_config_;
  SLBoundary adc_sl_boundary_;
  apollo::common::TrajectoryPoint init_point_;
  const ReferenceLine* reference_line_ = nullptr;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_TASKS_SPEED_DECIDER_SPEED_DECIDER_H_
