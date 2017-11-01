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
 * @file: pnc_map.h
 **/

#ifndef MODULES_MAP_PNC_MAP_PNC_MAP_H_
#define MODULES_MAP_PNC_MAP_PNC_MAP_H_

#include <limits>
#include <memory>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "gflags/gflags.h"

#include "modules/routing/proto/routing.pb.h"

#include "modules/map/hdmap/hdmap.h"
#include "modules/map/pnc_map/path.h"

namespace apollo {
namespace hdmap {

/**
 * @brief class RouteSegments
 *
 * This class is a representation of the Passage type in routing.proto.
 * It is exended from a passage region, but keeps some properties of the passage
 * such as the last end LaneWaypoint of the original passage region
 * (route_end_waypoint), whether the passage can lead to another passage in
 * routing (can_exit_). This class contains the original data that can be used
 * to generate hdmap::Path
 **/
class RouteSegments : public std::vector<LaneSegment> {
 public:
  /**
   * The default constructor.
   **/
  RouteSegments() = default;

  /**
   * Get the next change lane action need to take by the vehicle, if the vehicle
   * is on this RouteSegments. For a vehicle on this RouteSegment,
   * If the vehicle does not need to change lane, then change_lane_type =
   * routing::FORWARD
   * If the vehicle need to change to left lane according to routing, then
   * change_lane_type_ =  routing::LEFT;
   * If the vehicle need to change to right lane according to routing, then
   * change_lane_type_ = routing::RIGHT;
   */
  routing::ChangeLaneType NextAction() const;
  void SetNextAction(routing::ChangeLaneType action);

  /**
   * Wether the passage region that generate this route segment can lead to
   * another passage region in route.
   */
  bool CanExit() const;
  void SetCanExit(bool can_exit);

  /**
   * Project a point to this route segment.
   * @param point_enu a map point.
   * @param s return the longitudinal s relative to the route segment.
   * @param l return the lateral distance relative to the route segment.
   * @param waypoint return the LaneWaypoint, which has lane and lane_s on the
   * route segment.
   * @return false if error happended or projected outside of the lane segments.
   */
  bool GetProjection(const common::PointENU &point_enu, double *s, double *l,
                     LaneWaypoint *waypoint) const;

  /**
   * Check whether the map allows a vehicle can reach current RouteSegment from
   * a point on a lane (LaneWaypoint).
   * @param waypoint the start waypoint
   * @return true if the map allows a vehicle to drive from waypoint to current
   * RouteSegment. Otherwise false.
   */
  bool CanDriveFrom(const LaneWaypoint &waypoint) const;

  /*
   * This is the point that is the end of the original passage in routing.
   * It is used to check if the vehicle is out of current routing.
   * The LaneWaypoint.lane is nullptr if the end of the passage is not on the
   * RouteSegment.
   */
  const LaneWaypoint &RouteEndWaypoint() const;
  void SetRouteEndWaypoint(const LaneWaypoint &waypoint);

  bool IsOnSegment() const;
  void SetIsOnSegment(bool on_segment);

  /**
   * Get the last waypoint from the lane segments.
   */
  LaneWaypoint LastWaypoint() const;

  /**
   * @brief Check if a waypoint is on segment
   */
  bool IsWaypointOnSegment(const LaneWaypoint &waypoint) const;

 private:
  LaneWaypoint route_end_waypoint_;

  /**
   * wheter this segment can lead to another passage region in routing
   */
  bool can_exit_ = false;

  /**
   * Indicates whether the vehicle is on current RouteSegment.
   **/
  bool is_on_segment_ = false;

  routing::ChangeLaneType next_action_ = routing::FORWARD;
};

class PncMap {
 public:
  virtual ~PncMap() = default;
  explicit PncMap(const HDMap *hdmap);

  const hdmap::HDMap *hdmap() const;

  bool UpdateRoutingResponse(const routing::RoutingResponse &routing_response);
  bool UpdatePosition(const common::PointENU &point);

  const routing::RoutingResponse &routing_response() const;

  static bool CreatePathFromLaneSegments(const RouteSegments &segments,
                                         Path *const path);

  bool GetRouteSegments(const double backward_length,
                        const double forward_length,
                        std::vector<RouteSegments> *const route_segments) const;

 private:
  bool GetNearestPointFromRouting(const common::PointENU &point,
                                  LaneWaypoint *waypoint) const;

  /**
   * Find the waypoint index of a routing waypoint.
   * @return a vector with three values: Road index in RoutingResponse, Passage
   * index in RoadSegment, and segment index in a Passage. (-1, -1, -1) will be
   * returned if there is any error.
   */
  std::vector<int> GetWaypointIndex(const LaneWaypoint &waypoint) const;

  bool PassageToSegments(routing::Passage passage,
                         RouteSegments *segments) const;

  bool ProjectToSegments(const common::PointENU &point_enu,
                         const RouteSegments &segments,
                         LaneWaypoint *waypoint) const;

  bool TruncateLaneSegments(const RouteSegments &segments, double start_s,
                            double end_s,
                            RouteSegments *const truncated_segments) const;

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

 private:
  routing::RoutingResponse routing_;
  std::unordered_set<std::string> routing_lane_ids_;
  LaneWaypoint current_waypoint_;
  common::PointENU current_point_;
  std::vector<int> route_index_;
  common::PointENU passage_start_point_;
  double min_l_to_lane_center_ = std::numeric_limits<double>::max();
  const hdmap::HDMap *hdmap_ = nullptr;
};

}  // namespace hdmap
}  // namespace apollo

#endif  // MODULES_MAP_PNC_MAP_PNC_MAP_H_
