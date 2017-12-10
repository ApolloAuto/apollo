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
 * @file:
 **/

#ifndef MODULES_MAP_PNC_MAP_PNC_MAP_H_
#define MODULES_MAP_PNC_MAP_PNC_MAP_H_

#include <limits>
#include <list>
#include <memory>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "gflags/gflags.h"

#include "modules/common/proto/vehicle_state.pb.h"
#include "modules/routing/proto/routing.pb.h"

#include "modules/map/hdmap/hdmap.h"
#include "modules/map/pnc_map/path.h"
#include "modules/map/pnc_map/route_segments.h"

namespace apollo {
namespace hdmap {

class PncMap {
 public:
  virtual ~PncMap() = default;
  explicit PncMap(const HDMap *hdmap);

  const hdmap::HDMap *hdmap() const;

  bool UpdateRoutingResponse(const routing::RoutingResponse &routing_response);

  bool UpdateVehicleState(const common::VehicleState &vehicle_state);

  const routing::RoutingResponse &routing_response() const;

  static bool CreatePathFromLaneSegments(const RouteSegments &segments,
                                         Path *const path);

  bool GetRouteSegments(const double backward_length,
                        const double forward_length,
                        std::list<RouteSegments> *const route_segments) const;

  /**
   * Check if the routing is the same as existing one in PncMap
   */
  bool IsNewRouting(const routing::RoutingResponse &routing_response) const;

  bool ExtendSegments(const RouteSegments &segments,
                      const common::PointENU &point, double look_forward,
                      double look_backward, RouteSegments *extended_segments);

  bool ExtendSegments(const RouteSegments &segments, double start_s,
                      double end_s,
                      RouteSegments *const truncated_segments) const;

 private:
  /**
   * @brief Find the waypoint index of a routing waypoint. It updates
   * route_index_ with a vector with three values: Road index in
   * RoutingResponse, Passage index in RoadSegment, and segment index in a
   * Passage.
   * @return empty vector if cannot find waypoint on routing, otherwise
   *   a vector with three indices: {road_index, passage_index, lane_index}
   */
  std::vector<int> GetWaypointIndex(const LaneWaypoint &waypoint) const;

  bool GetNearestPointFromRouting(const common::VehicleState &point,
                                  LaneWaypoint *waypoint) const;

  bool PassageToSegments(routing::Passage passage,
                         RouteSegments *segments) const;

  bool ProjectToSegments(const common::PointENU &point_enu,
                         const RouteSegments &segments,
                         LaneWaypoint *waypoint) const;

  static bool ValidateRouting(const routing::RoutingResponse &routing);

  static void AppendLaneToPoints(LaneInfoConstPtr lane, const double start_s,
                                 const double end_s,
                                 std::vector<MapPathPoint> *const points);

  LaneInfoConstPtr GetRoutePredecessor(LaneInfoConstPtr lane) const;
  LaneInfoConstPtr GetRouteSuccessor(LaneInfoConstPtr lane) const;

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
  LaneWaypoint ToLaneWaypoint(const routing::LaneWaypoint &waypoint) const;

  /**
   * @brief Update routing waypoint index to the next waypoint that ADC need to
   * pass. The logic is by comparing the current waypoint's route index with
   * route_index and adc_waypoint_:
   * a. If the waypoint's route_index < route_index_, ADC must have passed
   * the waypoint.
   * b. If the waypoint's route_index == route_index_, ADC and the waypoint
   * is on the same lane, compare the lane_s.
   */
  void UpdateNextRoutingWaypointIndex();

 private:
  routing::RoutingResponse routing_;
  std::unordered_set<std::string> routing_lane_ids_;
  /**
   * The routing request waypoints
   */
  std::vector<LaneWaypoint> routing_waypoints_;
  /**
   * The next routing request waypoint that the vehicle have passed.
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
  std::vector<int> route_index_;
  /**
   * The waypoint of the autonomous driving car
   */
  LaneWaypoint adc_waypoint_;

  /**
   * @brief Indicates whether the adc should start consider destination.
   * In a looped routing, the vehicle may need to pass by the destination point
   * may times on the road, but only need to stop when it encounters destination
   * for the last time.
   */
  bool stop_for_destination_ = false;
};

}  // namespace hdmap
}  // namespace apollo

#endif  // MODULES_MAP_PNC_MAP_PNC_MAP_H_
