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

class ObjectsXmlParser {
 public:
  static Status ParseCrosswalks(const tinyxml2::XMLElement& xml_node,
                                std::vector<PbCrosswalk>* crosswalks);
  static Status ParseClearAreas(const tinyxml2::XMLElement& xml_node,
                                std::vector<PbClearArea>* clear_areas);
  static Status ParseSpeedBumps(const tinyxml2::XMLElement& xml_node,
                                std::vector<PbSpeedBump>* speed_bumps);
  static Status ParseStopLines(const tinyxml2::XMLElement& xml_node,
                               std::vector<StopLineInternal>* stop_lines);
  static Status ParseParkingSpaces(const tinyxml2::XMLElement& xml_node,
                                   std::vector<PbParkingSpace>* parking_spaces);
  static Status ParsePNCJunctions(const tinyxml2::XMLElement& xml_node,
                                  std::vector<PbPNCJunction>* pnc_junctions);
  static Status ParsePassageGroup(const tinyxml2::XMLElement& xml_node,
                                  PbPNCJunction* pnc_junction);

  static Status ParsePassage(const tinyxml2::XMLElement& xml_node,
                             PbPassageGroup* passage_group);
  static Status ParsePassageIds(const tinyxml2::XMLElement& xml_node,
                                const std::string& child_node_name,
                                std::vector<std::string>* passage_node_ids);
  static Status ToPassageType(const std::string& type,
                              PbPassageType* passage_type);
};

}  // namespace adapter
}  // namespace hdmap
}  // namespace apollo
