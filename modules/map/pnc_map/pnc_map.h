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
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "modules/routing/proto/routing.pb.h"

#include "modules/map/hdmap/hdmap.h"
#include "modules/map/pnc_map/path.h"

namespace apollo {
namespace hdmap {

using LaneSegments = std::vector<LaneSegment>;

class PncMap {
 public:
  virtual ~PncMap() = default;
  explicit PncMap(const HDMap *hdmap);

  const hdmap::HDMap *hdmap() const;

  bool UpdateRoutingResponse(const routing::RoutingResponse &routing_response);

  const routing::RoutingResponse &routing_response() const;

  bool GetLaneSegmentsFromRouting(
      const common::PointENU &point, const double backward_length,
      const double forward_length,
      std::vector<LaneSegments> *const route_segments) const;

  static bool CreatePathFromLaneSegments(const LaneSegments &segments,
                                         Path *const path);
  std::mutex reference_line_groups_mutex_;

 private:
  bool GetNearestPointFromRouting(const common::PointENU &point,
                                  LaneWaypoint *waypoint) const;

  bool TruncateLaneSegments(const LaneSegments &segments, double start_s,
                            double end_s,
                            LaneSegments *const truncated_segments) const;

  static bool ValidateRouting(const routing::RoutingResponse &routing);

  static void AppendLaneToPoints(LaneInfoConstPtr lane, const double start_s,
                                 const double end_s,
                                 std::vector<MapPathPoint> *const points);

 private:
  routing::RoutingResponse routing_;
  std::unique_ptr<LaneWaypoint> last_waypoint_;
  const hdmap::HDMap *hdmap_ = nullptr;
};

}  // namespace hdmap
}  // namespace apollo

#endif  // MODULES_MAP_PNC_MAP_PNC_MAP_H_
