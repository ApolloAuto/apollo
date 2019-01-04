/* Copyright 2017 The Apollo Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
=========================================================================*/
#pragma once

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "modules/common/util/util.h"

#include "modules/map/hdmap/adapter/xml_parser/common_define.h"
#include "modules/map/proto/map.pb.h"

namespace apollo {
namespace hdmap {
namespace adapter {

struct ProtoData {
  PbHeader header;
  std::unordered_map<std::string, PbLane> pb_lanes;
  std::unordered_map<std::string, PbRoad> pb_roads;
  std::unordered_map<std::string, PbCrosswalk> pb_crosswalks;
  std::unordered_map<std::string, PbClearArea> pb_clear_areas;
  std::unordered_map<std::string, PbSpeedBump> pb_speed_bumps;
  std::unordered_map<std::string, PbJunction> pb_junction;
  std::unordered_map<std::string, PbSignal> pb_signals;
  std::unordered_map<std::string, PbStopSign> pb_stop_signs;
  std::unordered_map<std::string, PbYieldSign> pb_yield_signs;
  std::unordered_map<std::string, PbOverlap> pb_overlaps;
  std::unordered_map<std::string, PbJunction> pb_junctions;
  std::unordered_map<std::string, StopLineInternal> pb_stop_lines;
  std::unordered_map<std::string, PbParkingSpace> pb_parking_spaces;
  std::unordered_map<std::string, PbPNCJunction> pb_pnc_junctions;
};

class ProtoOrganizer {
 public:
  void GetRoadElements(std::vector<RoadInternal>* roads);
  void GetJunctionElements(const std::vector<JunctionInternal>& junctions);
  void GetOverlapElements(const std::vector<RoadInternal>& roads,
                          const std::vector<JunctionInternal>& junctions);
  void OutputData(apollo::hdmap::Map* pb_map);

 private:
  void GetLaneObjectOverlapElements(
      const std::string& lane_id,
      const std::vector<OverlapWithLane>& overlap_with_lanes);
  void GetLaneSignalOverlapElements(
      const std::string& lane_id,
      const std::vector<OverlapWithLane>& overlap_with_lanes);
  void GetLaneJunctionOverlapElements(
      const std::string& lane_id,
      const std::vector<OverlapWithLane>& overlap_with_lanes);
  void GetLaneLaneOverlapElements(
      const std::unordered_map<std::pair<std::string, std::string>,
                               OverlapWithLane, apollo::common::util::PairHash>&
          lane_lane_overlaps);
  void GetJunctionObjectOverlapElements(
      const std::vector<JunctionInternal>& junctions);

 private:
  ProtoData proto_data_;
};

}  // namespace adapter
}  // namespace hdmap
}  // namespace apollo
