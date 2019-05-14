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
#include <vector>

#include "tinyxml2/tinyxml2.h"

#include "modules/map/hdmap/adapter/xml_parser/common_define.h"
#include "modules/map/hdmap/adapter/xml_parser/status.h"

namespace apollo {
namespace hdmap {
namespace adapter {

class LanesXmlParser {
 public:
  static Status Parse(const tinyxml2::XMLElement& xml_node,
                      const std::string& road_id,
                      std::vector<RoadSectionInternal>* sections);

 private:
  static Status ParseLaneSection(const tinyxml2::XMLElement& xml_node,
                                 std::vector<LaneInternal>* lanes);

  static Status ParseSectionBoundary(const tinyxml2::XMLElement& xml_node,
                                     PbBoundaryPolygon* boundary);

  static Status ToPbBoundaryType(const std::string& type,
                                 PbBoundaryEdgeType* boundary_type);
  static Status ParseLane(const tinyxml2::XMLElement& xml_node,
                          LaneInternal* lane_internal);
  static Status ParseDirection(const tinyxml2::XMLElement& xml_node,
                               PbLane* lane);
  static Status ParseCenterCurve(const tinyxml2::XMLElement& xml_node,
                                 PbLane* lane);
  static Status ParseSpeed(const tinyxml2::XMLElement& xml_node, PbLane* lane);
  static Status ParseSampleAssociates(const tinyxml2::XMLElement& xml_node,
                                      PbLane* lane);
  static Status ParseSingleSideRoadSampleAssociates(
      const tinyxml2::XMLElement& xml_node, bool bleft, PbLane* lane);
  static Status ParseLeftRoadSampleAssociates(
      const tinyxml2::XMLElement& xml_node, PbLane* lane);
  static Status ParseRightRoadSampleAssociates(
      const tinyxml2::XMLElement& xml_node, PbLane* lane);
  static Status ParseRoadSampleAssociates(const tinyxml2::XMLElement& xml_node,
                                          PbLane* lane);
  static Status ParseObjectOverlapGroup(
      const tinyxml2::XMLElement& xml_node,
      std::vector<OverlapWithLane>* object_overlaps);
  static Status ParseSignalOverlapGroup(
      const tinyxml2::XMLElement& xml_node,
      std::vector<OverlapWithLane>* signal_overlaps);
  static Status ParseJunctionOverlapGroup(
      const tinyxml2::XMLElement& xml_node,
      std::vector<OverlapWithLane>* junction_overlaps);
  static Status ParseLaneOverlapGroup(
      const tinyxml2::XMLElement& xml_node,
      std::vector<OverlapWithLane>* lane_overlaps);

  static Status ToPbLaneType(const std::string& type, PbLaneType* pb_type);
  static Status ToPbTurnType(const std::string& type, PbTurnType* pb_turn_type);
  static Status ToPbDirection(const std::string& type,
                              PbLaneDirection* pb_direction);

  static void ParseLaneLink(const tinyxml2::XMLElement& xml_node, PbLane* lane);
  static Status ParseLaneBorderMark(const tinyxml2::XMLElement& xml_node,
                                    PbLaneBoundaryTypeType* boundary_type);
  static Status ToPbLaneMarkType(const std::string& type,
                                 const std::string& color,
                                 PbLaneBoundaryTypeType* boundary_type);
  static Status ParseRegionOverlap(
      const tinyxml2::XMLElement& xml_node,
      std::vector<PbRegionOverlap>* region_overlaps);
};

}  // namespace adapter
}  // namespace hdmap
}  // namespace apollo
