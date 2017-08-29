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

#ifndef MODULES_DREAMVIEW_BACKEND_MAP_MAP_SERVICE_H_
#define MODULES_DREAMVIEW_BACKEND_MAP_MAP_SERVICE_H_

#include <string>
#include <vector>
#include "modules/map/pnc_map/pnc_map.h"
#include "third_party/json/json.hpp"

/**
 * @namespace apollo::dreamview
 * @brief apollo::dreamview
 */
namespace apollo {
namespace dreamview {

struct MapElementIds {
  std::vector<std::string> lane;
  std::vector<std::string> crosswalk;
  std::vector<std::string> junction;
  std::vector<std::string> signal;
  std::vector<std::string> stop_sign;
  std::vector<std::string> yield;
  std::vector<std::string> overlap;

  MapElementIds() = default;
  explicit MapElementIds(const nlohmann::json &json_object);

  void LogDebugInfo() const {
    AINFO << "Lanes: " << lane.size();
    AINFO << "Crosswalks: " << crosswalk.size();
    AINFO << "Junctions: " << junction.size();
    AINFO << "Signals: " << signal.size();
    AINFO << "StopSigns: " << stop_sign.size();
    AINFO << "YieldSigns: " << yield.size();
    AINFO << "Overlaps: " << overlap.size();
  }

  size_t Hash() const;
  nlohmann::json Json() const;
};

class MapService {
 public:
  explicit MapService(const std::string map_filename);

  MapService(const std::string &base_map_filename,
             const std::string &sim_map_filename);
  MapElementIds CollectMapElementIds(const apollo::common::PointENU &point,
                                     double raidus) const;

  bool GetPointsFromRouting(
      const apollo::routing::RoutingResponse &routing,
      std::vector<apollo::hdmap::MapPathPoint> *points) const;

  // The returned value is of a ::apollo::hdmap::Map proto. This
  // makes it easy to convert to a JSON object and to send to the
  // javascript clients.
  ::apollo::hdmap::Map RetrieveMapElements(const MapElementIds &ids) const;

  bool GetPoseWithRegardToLane(const double x, const double y, double *theta,
                               double *s) const;


  /**
   * @brief The function fills out proper routing lane waypoint
   * from the given (x,y) position.
   * @param x position
   * @param y position
   * @param laneWayPoint RoutingRequest's lane waypoint
   * @return True if the lane waypoint is filled successfully.
   */
  bool ConstructLaneWayPoint(
      const double x, const double y,
      ::apollo::routing::RoutingRequest::LaneWaypoint* laneWayPoint) const;

 private:
  const ::apollo::hdmap::HDMap &BaseMap() const {
    return pnc_map_.HDMap();
  }

  // A wrapper around HDMap to provide some convenient utils dreamview needs.
  const ::apollo::hdmap::PncMap pnc_map_;
  // A downsampled map for dreamview frontend display.
  ::apollo::hdmap::HDMap sim_map_;
};

}  // namespace dreamview
}  // namespace apollo

#endif  // MODULES_DREAMVIEW_BACKEND_MAP_MAP_SERVICE_H_
