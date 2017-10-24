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

#include <memory>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "modules/routing/proto/routing.pb.h"

#include "modules/map/hdmap/hdmap.h"
#include "modules/map/pnc_map/path.h"

namespace apollo {
namespace hdmap {

class RouteSegments : public std::vector<LaneSegment> {
 public:
  RouteSegments() = default;
  void SetChangeLaneType(routing::ChangeLaneType type) {
    change_lane_type_ = type;
  }
  routing::ChangeLaneType change_lane_type() const { return change_lane_type_; }

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

  bool CanDriveFrom(const LaneWaypoint &waypoint) const;

 private:
  bool IsOnSegment(const LaneWaypoint &waypoint) const;

  routing::ChangeLaneType change_lane_type_ = routing::FORWARD;
};

class PncMap {
 public:
  virtual ~PncMap() = default;
  explicit PncMap(const HDMap *hdmap);

  const hdmap::HDMap *hdmap() const;

  bool UpdateRoutingResponse(const routing::RoutingResponse &routing_response);

  const routing::RoutingResponse &routing_response() const;

  static bool CreatePathFromLaneSegments(const RouteSegments &segments,
                                         Path *const path);

  bool GetRouteSegments(const common::PointENU &point,
                        const double backward_length,
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

  std::vector<std::pair<int, routing::ChangeLaneType>> GetNeighborPassages(
      const routing::RoadSegment &road, int start_passage) const;

 private:
  routing::RoutingResponse routing_;
  std::unordered_set<std::string> routing_lane_ids_;
  std::unique_ptr<LaneWaypoint> last_waypoint_;
  const hdmap::HDMap *hdmap_ = nullptr;
};

}  // namespace hdmap
}  // namespace apollo

#endif  // MODULES_MAP_PNC_MAP_PNC_MAP_H_
