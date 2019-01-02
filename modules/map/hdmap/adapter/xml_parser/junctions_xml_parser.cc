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
#include "modules/map/hdmap/adapter/xml_parser/junctions_xml_parser.h"

#include <string>

#include "tinyxml2/tinyxml2.h"

#include "modules/map/hdmap/adapter/xml_parser/common_define.h"
#include "modules/map/hdmap/adapter/xml_parser/status.h"
#include "modules/map/hdmap/adapter/xml_parser/util_xml_parser.h"

namespace apollo {
namespace hdmap {
namespace adapter {

Status JunctionsXmlParser::Parse(const tinyxml2::XMLElement& xml_node,
                                 std::vector<JunctionInternal>* junctions) {
  const tinyxml2::XMLElement* junction_node =
      xml_node.FirstChildElement("junction");
  while (junction_node) {
    // id
    std::string junction_id;
    int checker =
        UtilXmlParser::QueryStringAttribute(*junction_node, "id", &junction_id);
    if (checker != tinyxml2::XML_SUCCESS) {
      std::string err_msg = "Error parse junction id";
      return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
    }

    // outline
    const tinyxml2::XMLElement* sub_node =
        junction_node->FirstChildElement("outline");
    if (!sub_node) {
      std::string err_msg = "Error parse junction outline";
      return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
    }

    PbJunction junction;
    junction.mutable_id()->set_id(junction_id);
    PbPolygon* polygon = junction.mutable_polygon();
    RETURN_IF_ERROR(UtilXmlParser::ParseOutline(*sub_node, polygon));

    JunctionInternal junction_internal;
    junction_internal.junction = junction;

    // overlap
    sub_node = junction_node->FirstChildElement("objectOverlapGroup");
    if (sub_node) {
      sub_node = sub_node->FirstChildElement("objectReference");
      while (sub_node) {
        std::string object_id;
        checker =
            UtilXmlParser::QueryStringAttribute(*sub_node, "id", &object_id);
        if (checker != tinyxml2::XML_SUCCESS) {
          std::string err_msg = "Error parse junction overlap id";
          return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
        }

        OverlapWithJunction overlap_with_juntion;
        overlap_with_juntion.object_id = object_id;
        junction_internal.overlap_with_junctions.push_back(
            overlap_with_juntion);

        sub_node = sub_node->NextSiblingElement("objectReference");
      }
    }

    junctions->push_back(junction_internal);
    junction_node = junction_node->NextSiblingElement("junction");
  }
  return Status::OK();
}

}  // namespace adapter
}  // namespace hdmap
}  // namespace apollo
