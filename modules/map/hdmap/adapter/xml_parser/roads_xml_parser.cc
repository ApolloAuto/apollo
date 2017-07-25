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
#include "modules/map/hdmap/adapter/xml_parser/roads_xml_parser.h"
#include <string>
#include <vector>
#include "modules/map/hdmap/adapter/xml_parser/lanes_xml_parser.h"
#include "modules/map/hdmap/adapter/xml_parser/objects_xml_parser.h"
#include "modules/map/hdmap/adapter/xml_parser/signals_xml_parser.h"
#include "modules/map/hdmap/adapter/xml_parser/util_xml_parser.h"
#include "glog/logging.h"

namespace apollo {
namespace hdmap {
namespace adapter {

Status RoadsXmlParser::parse(const tinyxml2::XMLElement& xml_node,
                            std::vector<Road>* roads) {
    CHECK_NOTNULL(roads);

    const tinyxml2::XMLElement* road_node = xml_node.FirstChildElement("road");
    while (road_node) {
        // road attributes
        // std::string unused_name;
        std::string id;
        std::string unused_junction;

        int checker = UtilXmlParser::query_string_attribute(*road_node,
                                                        "id", &id);
        checker += UtilXmlParser::query_string_attribute(*road_node,
                                                "junction", &unused_junction);

        if (checker != tinyxml2::XML_SUCCESS) {
            std::string err_msg = "Error parsing road attributes";
            return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
        }

        Road road;
        road.id = id;
        // lanes
        RETURN_IF_ERROR(LanesXmlParser::parse(*road_node, &road));

        // objects
        const tinyxml2::XMLElement* sub_node
                                    = road_node->FirstChildElement("objects");
        if (sub_node != nullptr) {
            // stop line
            ObjectsXmlParser::parse_stop_lines(*sub_node, &road.stop_lines);
            // crosswalks
            ObjectsXmlParser::parse_crosswalks(*sub_node, &road.crosswalks);
            // clearareas
            ObjectsXmlParser::parse_clear_areas(*sub_node, &road.clear_areas);
            // speed_bumps
            ObjectsXmlParser::parse_speed_bumps(*sub_node, &road.speed_bumps);
        }

        // signals
        sub_node = road_node->FirstChildElement("signals");
        if (sub_node != nullptr) {
            // traffic lights
            SignalsXmlParser::parse_traffic_lights(*sub_node,
                                                &road.traffic_lights);
            // stop signs
            SignalsXmlParser::parse_stop_signs(*sub_node, &road.stop_signs);
            // yield signs
            SignalsXmlParser::parse_yield_signs(*sub_node, &road.yield_signs);
        }

        roads->push_back(road);
        road_node = road_node->NextSiblingElement("road");
    }

    return Status::OK();
}

}  // namespace adapter
}  // namespace hdmap
}  // namespace apollo
