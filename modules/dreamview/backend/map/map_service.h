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

#include "boost/thread/locks.hpp"
#include "boost/thread/shared_mutex.hpp"

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
  explicit MapService(bool use_sim_map = true);

  MapElementIds CollectMapElementIds(const apollo::common::PointENU &point,
                                     double raidus) const;

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

  // Reload map from current FLAGS_map_dir.
  bool ReloadMap(bool force_reload);

 private:
  bool GetNearestLane(const double x, const double y,
                      apollo::hdmap::LaneInfoConstPtr *nearest_lane,
                      double *nearest_s, double *nearest_l) const;

  bool CreatePathsFromRouting(const routing::RoutingResponse &routing,
                              std::vector<apollo::hdmap::Path> *paths) const;

  bool AddPathFromPassageRegion(const routing::Passage &passage_region,
                                std::vector<apollo::hdmap::Path> *paths) const;

  const bool use_sim_map_;
  const hdmap::HDMap *hdmap_ = nullptr;
  // A downsampled map for dreamview frontend display.
  const hdmap::HDMap *sim_map_ = nullptr;
  bool pending = true;

  // RW lock to protect map data
  mutable boost::shared_mutex mutex_;
};

}  // namespace dreamview
}  // namespace apollo

#endif  // MODULES_DREAMVIEW_BACKEND_MAP_MAP_SERVICE_H_
