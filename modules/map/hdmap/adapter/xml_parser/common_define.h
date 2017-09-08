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
#ifndef MODULES_MAP_HDMAP_ADAPTER_XML_PARSER_COMMON_DEFINE_H_
#define MODULES_MAP_HDMAP_ADAPTER_XML_PARSER_COMMON_DEFINE_H_

#include <unordered_set>
#include <vector>
#include <string>
#include "modules/map/proto/map.pb.h"
#include "modules/common/log.h"

namespace apollo {
namespace hdmap {
namespace adapter {

typedef ::apollo::hdmap::Header        PbHeader;
typedef ::apollo::hdmap::Road          PbRoad;
typedef ::apollo::hdmap::RoadSection   PbRoadSection;
typedef ::apollo::hdmap::Lane          PbLane;
typedef ::apollo::hdmap::Junction      PbJunction;
typedef ::apollo::hdmap::Signal        PbSignal;
typedef ::apollo::hdmap::Subsignal     PbSubSignal;
typedef ::apollo::hdmap::Crosswalk     PbCrosswalk;
typedef ::apollo::hdmap::SpeedBump     PbSpeedBump;
typedef ::apollo::hdmap::StopSign      PbStopSign;
typedef ::apollo::hdmap::YieldSign     PbYieldSign;
typedef ::apollo::hdmap::ObjectOverlapInfo PbObjectOverlapInfo;
typedef ::apollo::hdmap::Overlap       PbOverlap;
typedef ::apollo::hdmap::ClearArea     PbClearArea;
typedef ::apollo::hdmap::LineSegment   PbLineSegment;
typedef ::apollo::hdmap::CurveSegment  PbCurveSegment;
typedef ::apollo::hdmap::Curve         PbCurve;
typedef ::apollo::common::PointENU         PbPoint3D;
typedef ::apollo::hdmap::Lane_LaneType PbLaneType;
typedef ::apollo::hdmap::Lane_LaneTurn PbTurnType;
typedef ::apollo::hdmap::Id            PbID;
typedef ::apollo::hdmap::LaneBoundary  PbLaneBoundary;
typedef ::apollo::hdmap::LaneBoundaryType_Type PbLaneBoundaryTypeType;
typedef ::apollo::hdmap::Polygon       PbPolygon;
typedef ::apollo::hdmap::BoundaryPolygon PbBoundaryPolygon;
typedef ::apollo::hdmap::BoundaryEdge  PbBoundaryEdge;

typedef ::apollo::hdmap::Lane_LaneDirection PbLaneDirection;
typedef ::apollo::hdmap::Signal_Type        PbSignalType;
typedef ::apollo::hdmap::Subsignal_Type     PbSubSignalType;
typedef ::apollo::hdmap::BoundaryEdge_Type  PbBoundaryEdgeType;

struct StopLineInternal {
  std::string id;
  PbCurve curve;
};

struct StopSignInternal {
  std::string id;
  PbStopSign stop_sign;
  std::unordered_set<std::string> stop_line_ids;
};

struct YieldSignInternal {
  std::string id;
  PbYieldSign yield_sign;
  std::unordered_set<std::string> stop_line_ids;
};

struct TrafficLightInternal {
  std::string id;
  PbSignal traffic_light;
  std::unordered_set<std::string> stop_line_ids;
};

struct OverlapWithLane {
  std::string object_id;
  double start_s;
  double end_s;
  bool is_merge;

  OverlapWithLane() : is_merge(false) {}
};

struct OverlapWithJunction {
  std::string object_id;
};

struct LaneInternal {
  PbLane lane;
  std::vector<OverlapWithLane> overlap_signals;
  std::vector<OverlapWithLane> overlap_objects;
  std::vector<OverlapWithLane> overlap_junctions;
  // somewhat trick
  std::vector<OverlapWithLane> overlap_lanes;
};

struct JunctionInternal {
  PbJunction junction;
  std::unordered_set<std::string> road_ids;
  std::vector<OverlapWithJunction> overlap_with_junctions;
};

struct RoadSectionInternal {
  std::string id;
  PbRoadSection section;
  std::vector<LaneInternal> lanes;
};

struct RoadInternal {
  std::string id;
  PbRoad road;

  bool in_junction;
  std::string junction_id;

  std::vector<RoadSectionInternal> sections;

  std::vector<TrafficLightInternal> traffic_lights;
  std::vector<StopSignInternal> stop_signs;
  std::vector<YieldSignInternal> yield_signs;
  std::vector<PbCrosswalk> crosswalks;
  std::vector<PbClearArea> clear_areas;
  std::vector<PbSpeedBump> speed_bumps;
  std::vector<StopLineInternal> stop_lines;

  RoadInternal() : in_junction(false) {
    junction_id = "";
  }
};

}  // namespace adapter
}  // namespace hdmap
}  // namespace apollo

#endif  // MODULES_MAP_HDMAP_ADAPTER_XML_PARSER_COMMON_DEFINE_H_
