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
  PncMap() = default;
  virtual ~PncMap() = default;
  explicit PncMap(const std::string &map_file);

  const hdmap::HDMap& HDMap() const;

  /**
   * @brief Warning: this function only works if there is no change lane in
   *routing.
   **/
  bool CreatePathFromRouting(const ::apollo::routing::RoutingResponse &routing,
                             Path *const path) const;

  bool GetLaneSegmentsFromRouting(
      const ::apollo::routing::RoutingResponse &routing,
      const common::PointENU &point,
      const double backward_length, const double forward_length,
      std::vector<LaneSegments> *const route_segments) const;

  static bool CreatePathFromLaneSegments(const LaneSegments &segments,
                                         Path *const path);

 private:
  bool TruncateLaneSegments(const LaneSegments &segments,
                            double start_s, double end_s,
                            LaneSegments *const truncated_segments) const;

  bool ValidateRouting(const ::apollo::routing::RoutingResponse &routing) const;
  static void AppendLaneToPoints(
      LaneInfoConstPtr lane, const double start_s, const double end_s,
      std::vector<MapPathPoint> *const points);
  hdmap::HDMap hdmap_;
};

}  // namespace hdmap
}  // namespace apollo

#endif  // MODULES_MAP_PNC_MAP_PNC_MAP_H_
