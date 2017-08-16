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
#ifndef MODULES_MAP_MAP_LOADER_ADAPTER_PROTO_REORGANIZE_H
#define MODULES_MAP_MAP_LOADER_ADAPTER_PROTO_REORGANIZE_H

#include <unordered_map>
#include <unordered_set>
#include <string>
#include <vector>
#include "modules/map/proto/map.pb.h"
#include "modules/map/hdmap/adapter/xml_parser/common_define.h"

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
};

class ProtoOrganization {
 public:
  ProtoOrganization();
  ~ProtoOrganization();

 public:
  void get_road_elements(std::vector<RoadInternal>* roads,
                        ProtoData* proto_data);
  void get_junction_elements(const std::vector<JunctionInternal>& junctions,
                        ProtoData* proto_data);
  void get_overlap_elements(const std::vector<RoadInternal>& roads,
                        const std::vector<JunctionInternal>& junctions,
                        ProtoData* proto_data);
  void output_data(const ProtoData& proto_data, ::apollo::hdmap::Map* pb_map);

 private:
  ProtoData   _pb_map_data;
};

}  // namespace adapter
}  // namespace hdmap
}  // namespace apollo

#endif  // MODULES_MAP_MAP_LOADER_ADAPTER_PROTO_REORGANIZE_H
