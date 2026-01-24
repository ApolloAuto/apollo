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
 * @file lane_follow_map.h
 **/

#pragma once

#include <list>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"
#include "modules/common_msgs/routing_msgs/routing.pb.h"

#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/map/hdmap/hdmap.h"
#include "modules/map/pnc_map/path.h"
#include "modules/map/pnc_map/pnc_map_base.h"
#include "modules/map/pnc_map/route_segments.h"

namespace apollo {
namespace planning {

class LaneFollowMap : public PncMapBase {
 public:
  LaneFollowMap();

  virtual ~LaneFollowMap() = default;

  bool CanProcess(const planning::PlanningCommand &command) const override;

  bool UpdatePlanningCommand(const planning::PlanningCommand &command) override;
  /**
   * @brief use heuristic forward length and backward length
   */
  bool GetRouteSegments(
      const common::VehicleState &vehicle_state,
      std::list<apollo::hdmap::RouteSegments> *const route_segments) override;

  bool ExtendSegments(
      const apollo::hdmap::RouteSegments &segments, double start_s,
      double end_s,
      apollo::hdmap::RouteSegments *const truncated_segments) const override;

  std::vector<routing::LaneWaypoint> FutureRouteWaypoints() const override;
  /**
   * @brief Get the end point of PlanningCommand.
   * @param end_point The end point of PlanningCommand.
   */
  void GetEndLaneWayPoint(
      std::shared_ptr<routing::LaneWaypoint> &end_point) const override;
  /**
   * @brief Get the Lane with the given id.
   * @param id The id of the lane.
   */
  hdmap::LaneInfoConstPtr GetLaneById(const hdmap::Id &id) const override;

  bool GetNearestPointFromRouting(
      const common::VehicleState &state,
      apollo::hdmap::LaneWaypoint *waypoint) const override;
  
  double GetDistanceToDestination() const override;
  apollo::hdmap::LaneWaypoint GetAdcWaypoint() const override;

 private:
  /**
   * @brief Check if the command can be processed by this map.
   * @param command The command to be checked.
   * @return True if the command can be processed.
   */
  bool IsValid(const planning::PlanningCommand &command) const override;

  bool GetRouteSegments(
      const common::VehicleState &vehicle_state, const double backward_length,
      const double forward_length,
      std::list<apollo::hdmap::RouteSegments> *const route_segments);

  bool ExtendSegments(const apollo::hdmap::RouteSegments &segments,
                      const common::PointENU &point, double look_forward,
                      double look_backward,
                      apollo::hdmap::RouteSegments *extended_segments);

  bool UpdateVehicleState(const common::VehicleState &vehicle_state);
  /**
   * @brief Find the waypoint index of a routing waypoint. It updates
   * adc_route_index_
   * @return index out of range if cannot find waypoint on routing, otherwise
   *   an index in range [0, route_indices.size());
   */
  int GetWaypointIndex(const apollo::hdmap::LaneWaypoint &waypoint) const;

  bool PassageToSegments(routing::Passage passage,
                         apollo::hdmap::RouteSegments *segments) const;

  bool ProjectToSegments(const common::PointENU &point_enu,
                         const apollo::hdmap::RouteSegments &segments,
                         apollo::hdmap::LaneWaypoint *waypoint) const;

  static void AppendLaneToPoints(
      apollo::hdmap::LaneInfoConstPtr lane, const double start_s,
      const double end_s,
      std::vector<apollo::hdmap::MapPathPoint> *const points);

  apollo::hdmap::LaneInfoConstPtr GetRoutePredecessor(
      apollo::hdmap::LaneInfoConstPtr lane) const;
  apollo::hdmap::LaneInfoConstPtr GetRouteSuccessor(
      apollo::hdmap::LaneInfoConstPtr lane) const;

  /**
   * Return the neighbor passages from passage with index start_passage on road.
   * @param road the road information from routing
   * @param start_passage the passsage index in road
   * @return all the indices of the neighboring passages, including
   * start_passage.
   */
  std::vector<int> GetNeighborPassages(const routing::RoadSegment &road,
                                       int start_passage) const;

  /**
   * @brief convert a routing waypoint to lane waypoint
   * @return empty LaneWaypoint if the lane id cannot be found on map, otherwise
   * return a valid LaneWaypoint with lane ptr and s.
   */
  apollo::hdmap::LaneWaypoint ToLaneWaypoint(
      const routing::LaneWaypoint &waypoint) const;

  /**
   * @brief convert a routing segment to lane segment
   * @return empty LaneSegmetn if the lane id cannot be found on map, otherwise
   * return a valid LaneSegment with lane ptr, start_s and end_s
   */
  apollo::hdmap::LaneSegment ToLaneSegment(
      const routing::LaneSegment &segment) const;

  /**
   * @brief Update routing waypoint index to the next waypoint that ADC need to
   * pass. The logic is by comparing the current waypoint's route index with
   * route_index and adc_waypoint_:
   * a. If the waypoint's route_index < route_index_, ADC must have passed
   * the waypoint.
   * b. If the waypoint's route_index == route_index_, ADC and the waypoint
   * is on the same lane, compare the lane_s.
   */
  void UpdateNextRoutingWaypointIndex(int cur_index);

  /**
   * @brief find the index of waypoint by looking forward from index start.
   * @return empty vector if not found, otherwise return a vector { road_index,
   * passage_index, lane_index}
   */
  int SearchForwardWaypointIndex(
      int start, const apollo::hdmap::LaneWaypoint &waypoint) const;
  int SearchBackwardWaypointIndex(
      int start, const apollo::hdmap::LaneWaypoint &waypoint) const;

  void UpdateRoutingRange(int adc_index);
  void UpdateRouteSegmentsLaneIds(
      const std::list<hdmap::RouteSegments> *route_segments);

 private:
  struct RouteIndex {
    apollo::hdmap::LaneSegment segment;
    std::array<int, 3> index;
  };
  std::vector<RouteIndex> route_indices_;
  int range_start_ = 0;
  int range_end_ = 0;
  // routing ids in range
  std::unordered_set<std::string> range_lane_ids_;
  std::unordered_set<std::string> all_lane_ids_;
  std::unordered_set<std::string> route_segments_lane_ids_;
  routing::LaneSegment dest_lane_segment_;

  /**
   * The routing request waypoints
   */
  struct WaypointIndex {
    apollo::hdmap::LaneWaypoint waypoint;
    int index;
    WaypointIndex(const apollo::hdmap::LaneWaypoint &waypoint, int index)
        : waypoint(waypoint), index(index) {}
  };

  // return the segment of an index
  int NextWaypointIndex(int index) const;

  std::vector<WaypointIndex> routing_waypoint_index_;
  /**
   * The next routing request waypoint index in routing_waypoint_index_
   */
  std::size_t next_routing_waypoint_index_ = 0;

  const hdmap::HDMap *hdmap_ = nullptr;
  bool is_same_routing_ = false;

  /**
   * The state of the adc
   */
  common::VehicleState adc_state_;
  /**
   * A three element index: {road_index, passage_index, lane_index}
   */
  int adc_route_index_ = -1;
  /**
   * @brief Indicates whether the adc should start consider destination.
   * In a looped routing, the vehicle may need to pass by the destination
   * point
   * may times on the road, but only need to stop when it encounters
   * destination
   * for the last time.
   */
  bool stop_for_destination_ = false;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::LaneFollowMap,
                                     PncMapBase)

}  // namespace planning
}  // namespace apollo
