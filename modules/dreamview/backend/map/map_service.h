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
 */

#pragma once

#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <string>
#include <vector>

#include "modules/dreamview/proto/simulation_world.pb.h"
#include "modules/map/pnc_map/pnc_map.h"
#include "third_party/json/json.hpp"

/**
 * @namespace apollo::dreamview
 * @brief apollo::dreamview
 */
namespace apollo {
namespace dreamview {

class MapService {
 public:
  explicit MapService(bool use_sim_map = true);

  inline double GetXOffset() const { return x_offset_; }
  inline double GetYOffset() const { return y_offset_; }

  void CollectMapElementIds(const apollo::common::PointENU &point,
                            double raidus, MapElementIds *ids) const;

  bool GetPathsFromRouting(const apollo::routing::RoutingResponse &routing,
                           std::vector<apollo::hdmap::Path> *paths) const;

  // The returned value is of a hdmap::Map proto. This
  // makes it easy to convert to a JSON object and to send to the
  // javascript clients.
  hdmap::Map RetrieveMapElements(const MapElementIds &ids) const;

  bool GetPoseWithRegardToLane(const double x, const double y, double *theta,
                               double *s) const;

  // Get a point on the map to serve as dummy start point of SimControl
  bool GetStartPoint(apollo::common::PointENU *start_point) const;

  /**
   * @brief The function fills out proper routing lane waypoint
   * from the given (x,y) position.
   * @param x position
   * @param y position
   * @param laneWayPoint RoutingRequest's lane waypoint
   * @return True if the lane waypoint is filled successfully.
   */
  bool ConstructLaneWayPoint(const double x, const double y,
                             routing::LaneWaypoint *laneWayPoint) const;

  bool ConstructLaneWayPointWithHeading(
      const double x, const double y, const double heading,
      routing::LaneWaypoint *laneWayPoint) const;

  bool CheckRoutingPoint(const double x, const double y) const;

  bool CheckRoutingPointLaneType(apollo::hdmap::LaneInfoConstPtr lane) const;

  // Reload map from current FLAGS_map_dir.
  bool ReloadMap(bool force_reload);

  size_t CalculateMapHash(const MapElementIds &ids) const;

 private:
  void UpdateOffsets();
  bool GetNearestLane(const double x, const double y,
                      apollo::hdmap::LaneInfoConstPtr *nearest_lane,
                      double *nearest_s, double *nearest_l) const;

  bool GetNearestLaneWithHeading(const double x, const double y,
                                 apollo::hdmap::LaneInfoConstPtr *nearest_lane,
                                 double *nearest_s, double *nearest_l,
                                 const double heading) const;

  bool CreatePathsFromRouting(const routing::RoutingResponse &routing,
                              std::vector<apollo::hdmap::Path> *paths) const;

  bool AddPathFromPassageRegion(const routing::Passage &passage_region,
                                std::vector<apollo::hdmap::Path> *paths) const;

  static const char kMetaFileName[];

  const bool use_sim_map_;
  const hdmap::HDMap *HDMap() const;
  // A downsampled map for dreamview frontend display.
  const hdmap::HDMap *SimMap() const;
  bool MapReady() const;

  double x_offset_ = 0.0;
  double y_offset_ = 0.0;

  // RW lock to protect map data
  mutable boost::shared_mutex mutex_;
};

}  // namespace dreamview
}  // namespace apollo
