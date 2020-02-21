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
#include <string>

#include "modules/map/hdmap/adapter/xml_parser/lanes_xml_parser.h"
#include "modules/map/hdmap/adapter/xml_parser/objects_xml_parser.h"
#include "modules/map/hdmap/adapter/xml_parser/roads_xml_parser.h"
#include "modules/map/hdmap/adapter/xml_parser/signals_xml_parser.h"
#include "modules/map/hdmap/adapter/xml_parser/util_xml_parser.h"

namespace {
bool IsRoadBelongToJunction(const std::string& road_id) {
  ACHECK(!road_id.empty());
  return road_id != "-1";
}
}  // namespace

namespace apollo {
namespace hdmap {
namespace adapter {

Status RoadsXmlParser::Parse(const tinyxml2::XMLElement& xml_node,
                             std::vector<RoadInternal>* roads) {
  CHECK_NOTNULL(roads);

  auto road_node = xml_node.FirstChildElement("road");
  while (road_node) {
    // road attributes
    std::string id;
    std::string junction_id;
    int checker = UtilXmlParser::QueryStringAttribute(*road_node, "id", &id);
    checker += UtilXmlParser::QueryStringAttribute(*road_node, "junction",
                                                   &junction_id);
    if (checker != tinyxml2::XML_SUCCESS) {
      std::string err_msg = "Error parsing road attributes";
      return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
    }

    RoadInternal road_internal;
    road_internal.id = id;
    road_internal.road.mutable_id()->set_id(id);
    if (IsRoadBelongToJunction(junction_id)) {
      road_internal.road.mutable_junction_id()->set_id(junction_id);
    }

    std::string type;
    checker = UtilXmlParser::QueryStringAttribute(*road_node, "type", &type);
    if (checker != tinyxml2::XML_SUCCESS) {
      // forward compatibility with old data
      type = "CITYROAD";
    }
    PbRoadType pb_road_type;
    RETURN_IF_ERROR(to_pb_road_type(type, &pb_road_type));
    road_internal.road.set_type(pb_road_type);

    // lanes
    RETURN_IF_ERROR(LanesXmlParser::Parse(*road_node, road_internal.id,
                                          &road_internal.sections));

    // objects
    Parse_road_objects(*road_node, &road_internal);
    // signals
    Parse_road_signals(*road_node, &road_internal);

    roads->push_back(road_internal);
    road_node = road_node->NextSiblingElement("road");
  }

  return Status::OK();
}

void RoadsXmlParser::Parse_road_objects(const tinyxml2::XMLElement& xml_node,
                                        RoadInternal* road_info) {
  CHECK_NOTNULL(road_info);

  // objects
  auto sub_node = xml_node.FirstChildElement("objects");
  if (sub_node != nullptr) {
    // stop line
    ObjectsXmlParser::ParseStopLines(*sub_node, &road_info->stop_lines);
    // crosswalks
    ObjectsXmlParser::ParseCrosswalks(*sub_node, &road_info->crosswalks);
    // clearareas
    ObjectsXmlParser::ParseClearAreas(*sub_node, &road_info->clear_areas);
    // speed_bumps
    ObjectsXmlParser::ParseSpeedBumps(*sub_node, &road_info->speed_bumps);
    // parking_spaces
    ObjectsXmlParser::ParseParkingSpaces(*sub_node, &road_info->parking_spaces);
    // pnc_junctions
    ObjectsXmlParser::ParsePNCJunctions(*sub_node, &road_info->pnc_junctions);
  }
}

void RoadsXmlParser::Parse_road_signals(const tinyxml2::XMLElement& xml_node,
                                        RoadInternal* road_info) {
  CHECK_NOTNULL(road_info);

  // signals
  auto sub_node = xml_node.FirstChildElement("signals");
  if (sub_node != nullptr) {
    // traffic lights
    SignalsXmlParser::ParseTrafficLights(*sub_node, &road_info->traffic_lights);
    // stop signs
    SignalsXmlParser::ParseStopSigns(*sub_node, &road_info->stop_signs);
    // yield signs
    SignalsXmlParser::ParseYieldSigns(*sub_node, &road_info->yield_signs);
  }
}

Status RoadsXmlParser::to_pb_road_type(const std::string& type,
                                       PbRoadType* pb_road_type) {
  CHECK_NOTNULL(pb_road_type);

  std::string upper_type = UtilXmlParser::ToUpper(type);

  if (upper_type == "UNKNOWN") {
    *pb_road_type = apollo::hdmap::Road::UNKNOWN;
  } else if (upper_type == "HIGHWAY") {
    *pb_road_type = apollo::hdmap::Road::HIGHWAY;
  } else if (upper_type == "CITYROAD") {
    *pb_road_type = apollo::hdmap::Road::CITY_ROAD;
  } else if (upper_type == "PARK") {
    *pb_road_type = apollo::hdmap::Road::PARK;
  } else {
    std::string err_msg = "Error or unsupport road type";
    return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
  }

  return Status::OK();
}

}  // namespace adapter
}  // namespace hdmap
}  // namespace apollo
