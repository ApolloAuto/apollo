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
#include <glog/logging.h>
#include "modules/map/hdmap/adapter/xml_parser/status.h"

namespace apollo {
namespace hdmap {
namespace adapter {

int OpendriveAdapter::load_data(const std::string& filename,
                                apollo::hdmap::Map* pb_map) {
  CHECK_NOTNULL(pb_map);

  tinyxml2::XMLDocument document;
  int checker = document.LoadFile(filename.c_str());
  if (checker != tinyxml2::XML_SUCCESS) {
    return -1;
  }

  // root node;
  const tinyxml2::XMLElement* root_node = document.RootElement();
  // header
  PbHeader* map_header = pb_map->mutable_header();
  Status status = HeaderXmlParser::parse(*root_node, map_header);
  if (!status.ok()) {
    return -1;
  }

  // roads
  std::vector<RoadInternal> roads;
  status = RoadsXmlParser::parse(*root_node, &roads);
  if (!status.ok()) {
    return -1;
  }

  // junction
  std::vector<JunctionInternal> junctions;
  status = JunctionsXmlParser::parse(*root_node, &junctions);
  if (!status.ok()) {
    return -1;
  }

  ProtoData proto_data;
  ProtoOrganization proto_organizer;
  proto_organizer.get_road_elements(&roads, &proto_data);
  proto_organizer.get_junction_elements(junctions, &proto_data);
  proto_organizer.get_overlap_elements(roads, junctions, &proto_data);
  proto_organizer.output_data(proto_data, pb_map);

  return 0;
}

}  // namespace adapter
}  // namespace hdmap
}  // namespace apollo
