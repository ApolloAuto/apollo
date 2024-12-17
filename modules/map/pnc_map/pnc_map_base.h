/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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
 * @file planning_map_base.h
 */

#pragma once

#include <list>
#include <memory>
#include <vector>

#include "gflags/gflags.h"

#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"
#include "modules/common_msgs/planning_msgs/planning_command.pb.h"
#include "modules/common_msgs/routing_msgs/routing.pb.h"

#include "modules/map/pnc_map/route_segments.h"

namespace apollo {
namespace planning {

DECLARE_double(look_backward_distance);
DECLARE_double(look_forward_short_distance);
DECLARE_double(look_forward_long_distance);

/**
 * @class ExternalCommandProcessComponent
 *
 * @brief The external interface for processing external commands.
 */
class PncMapBase {
 public:
  virtual ~PncMapBase() = default;

  virtual bool CanProcess(const planning::PlanningCommand &command) const = 0;

  virtual bool UpdatePlanningCommand(const planning::PlanningCommand &command);

  static double LookForwardDistance(const double velocity);
  /**
   * @brief use heuristic forward length and backward length
   */
  virtual bool GetRouteSegments(
      const common::VehicleState &vehicle_state,
      std::list<apollo::hdmap::RouteSegments> *const route_segments) = 0;

  /**
   * Check if the routing is the same as existing one in Map
   */
  bool IsNewPlanningCommand(const planning::PlanningCommand &command) const;

  static bool IsNewPlanningCommand(
      const planning::PlanningCommand &prev_command,
      const planning::PlanningCommand &new_command);

  virtual bool ExtendSegments(
      const apollo::hdmap::RouteSegments &segments, double start_s,
      double end_s,
      apollo::hdmap::RouteSegments *const truncated_segments) const = 0;

  virtual std::vector<routing::LaneWaypoint> FutureRouteWaypoints() const = 0;
  /**
   * @brief Get the end point of PlanningCommand.
   * @param end_point The end point of PlanningCommand.
   */
  virtual void GetEndLaneWayPoint(
      std::shared_ptr<routing::LaneWaypoint> &end_point) const = 0;
  /**
   * @brief Get the Lane with the given id.
   * @param id The id of the lane.
   */
  virtual hdmap::LaneInfoConstPtr GetLaneById(const hdmap::Id &id) const = 0;

  virtual bool GetNearestPointFromRouting(
      const common::VehicleState &state,
      apollo::hdmap::LaneWaypoint *waypoint) const = 0;

  virtual double GetDistanceToDestination() const = 0;
  virtual apollo::hdmap::LaneWaypoint GetAdcWaypoint() const = 0;

 protected:
  planning::PlanningCommand last_command_;
  /**
   * The waypoint of the autonomous driving car
   */
  apollo::hdmap::LaneWaypoint adc_waypoint_;

 private:
  /**
   * @brief Check if the command can be processed by this map.
   * @param command The command to be checked.
   * @return True if the command can be processed.
   */
  virtual bool IsValid(const planning::PlanningCommand &command) const = 0;
};

}  // namespace planning
}  // namespace apollo
