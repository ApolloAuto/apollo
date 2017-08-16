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
#ifndef MODULES_MAP_MAP_LOADER_ADAPTER_XML_PARSER_LANE_XML_PARSER_H
#define MODULES_MAP_MAP_LOADER_ADAPTER_XML_PARSER_LANE_XML_PARSER_H

#include <vector>
#include <string>
#include "modules/map/hdmap/adapter/xml_parser/common_define.h"
#include "tinyxml2.h"
#include "modules/map/hdmap/adapter/xml_parser/status.h"

namespace apollo {
namespace hdmap {
namespace adapter {

class LanesXmlParser {
 public:
  static Status parse(const tinyxml2::XMLElement& xml_node,
                    const std::string& road_id,
                    std::vector<RoadSectionInternal> *sections);
  static Status parse_lane_section(const tinyxml2::XMLElement& xml_node,
                      const std::string& road_id,
                      const std::string& section_id,
                      std::vector<LaneInternal>* lanes);

  static Status parse_section_boundary(const tinyxml2::XMLElement& xml_node,
                                    PbBoundaryPolygon* boundary);

  static Status to_pb_boundary_type(const std::string& type,
                                 PbBoundaryEdgeType* boundary_type);
  static Status parse_lane(const tinyxml2::XMLElement& xml_node,
                          const std::string& road_id,
                          const std::string& section_id,
                          LaneInternal* lane_internal);
  static Status parse_direction(const tinyxml2::XMLElement& xml_node,
                            PbLane* lane);
  static Status parse_center_curve(const tinyxml2::XMLElement& xml_node,
                                PbLane* lane);
  static Status parse_speed(const tinyxml2::XMLElement& xml_node,
                            PbLane* lane);
  static Status parse_sample_associates(const tinyxml2::XMLElement& xml_node,
                                        PbLane* lane);
  static Status parse_object_overlap_group(const tinyxml2::XMLElement& xml_node,
                                std::vector<OverlapWithLane>* object_overlaps);
  static Status parse_signal_overlap_group(const tinyxml2::XMLElement& xml_node,
                                std::vector<OverlapWithLane>* signal_overlaps);
  static Status parse_junction_overlap_group(
                              const tinyxml2::XMLElement& xml_node,
                              std::vector<OverlapWithLane>* junction_overlaps);
  static Status parse_lane_overlap_group(const tinyxml2::XMLElement& xml_node,
                                  std::vector<OverlapWithLane>* lane_overlaps);

  static Status to_pb_lane_type(const std::string& type,
                              PbLaneType* pb_type);
  static Status to_pb_turn_type(const std::string& type,
                              PbTurnType* pb_turn_type);
  static Status to_pb_dirction(const std::string& type,
                              PbLaneDirection* pb_direction);

  static void parse_lane_link(const tinyxml2::XMLElement& xml_node,
                              const std::string& /*road_id*/,
                              const std::string& /*section_id*/,
                              PbLane* lane);
  static Status parse_lane_border_mark(const tinyxml2::XMLElement& xml_node,
                              PbLaneBoundaryTypeType* boundary_type);
  static Status to_pb_lane_mark_type(const std::string& type,
                              const std::string& color,
                              PbLaneBoundaryTypeType* boundary_type);
};

}  // namespace adapter
}  // namespace hdmap
}  // namespace apollo

#endif  // MODULES_MAP_MAP_LOADER_ADAPTER_XML_PARSER_LANE_XML_PARSER_H
