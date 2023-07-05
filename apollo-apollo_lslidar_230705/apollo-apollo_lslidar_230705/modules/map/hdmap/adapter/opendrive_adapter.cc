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
#include "modules/map/hdmap/adapter/opendrive_adapter.h"

#include <vector>

#include "cyber/common/log.h"
#include "modules/map/hdmap/adapter/proto_organizer.h"
#include "modules/map/hdmap/adapter/xml_parser/common_define.h"
#include "modules/map/hdmap/adapter/xml_parser/coordinate_convert_tool.h"
#include "modules/map/hdmap/adapter/xml_parser/header_xml_parser.h"
#include "modules/map/hdmap/adapter/xml_parser/junctions_xml_parser.h"
#include "modules/map/hdmap/adapter/xml_parser/lanes_xml_parser.h"
#include "modules/map/hdmap/adapter/xml_parser/objects_xml_parser.h"
#include "modules/map/hdmap/adapter/xml_parser/roads_xml_parser.h"
#include "modules/map/hdmap/adapter/xml_parser/signals_xml_parser.h"
#include "modules/map/hdmap/adapter/xml_parser/status.h"

namespace apollo {
namespace hdmap {
namespace adapter {

bool OpendriveAdapter::LoadData(const std::string& filename,
                                apollo::hdmap::Map* pb_map) {
  CHECK_NOTNULL(pb_map);

  tinyxml2::XMLDocument document;
  if (document.LoadFile(filename.c_str()) != tinyxml2::XML_SUCCESS) {
    AERROR << "fail to load file " << filename;
    return false;
  }

  // root node
  const tinyxml2::XMLElement* root_node = document.RootElement();
  ACHECK(root_node != nullptr);
  // header
  PbHeader* map_header = pb_map->mutable_header();
  Status status = HeaderXmlParser::Parse(*root_node, map_header);
  if (!status.ok()) {
    AERROR << "fail to parse opendrive header, " << status.error_message();
    return false;
  }

  // road
  std::vector<RoadInternal> roads;
  status = RoadsXmlParser::Parse(*root_node, &roads);
  if (!status.ok()) {
    AERROR << "fail to parse opendrive road, " << status.error_message();
    return false;
  }

  // junction
  std::vector<JunctionInternal> junctions;
  status = JunctionsXmlParser::Parse(*root_node, &junctions);
  if (!status.ok()) {
    AERROR << "fail to parse opendrive junction, " << status.error_message();
    return false;
  }

  // objects
  ObjectInternal objects;
  status = ObjectsXmlParser::ParseObjects(*root_node, &objects);
  if (!status.ok()) {
    AERROR << "fail to parse opendrive objects, " << status.error_message();
    return false;
  }

  ProtoOrganizer proto_organizer;
  proto_organizer.GetRoadElements(&roads);
  proto_organizer.GetJunctionElements(junctions);
  proto_organizer.GetObjectElements(objects);
  proto_organizer.GetOverlapElements(roads, junctions);
  proto_organizer.OutputData(pb_map);

  return true;
}

}  // namespace adapter
}  // namespace hdmap
}  // namespace apollo
