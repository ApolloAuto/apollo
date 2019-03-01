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

#include "modules/map/hdmap/adapter/xml_parser/header_xml_parser.h"

#include <string>

#include "modules/map/hdmap/adapter/coordinate_convert_tool.h"
#include "modules/map/hdmap/adapter/xml_parser/util_xml_parser.h"

namespace {
int GetLongZone(double longitude) {
  double longZone = 0.0;
  if (longitude < 0.0) {
    longZone = ((180.0 + longitude) / 6.0) + 1;
  } else {
    longZone = (longitude / 6.0) + 31;
  }
  return static_cast<int>(longZone);
}
}  // namespace

namespace apollo {
namespace hdmap {
namespace adapter {

Status HeaderXmlParser::Parse(const tinyxml2::XMLElement& xml_node,
                              PbHeader* header) {
  auto header_node = xml_node.FirstChildElement("header");
  if (!header_node) {
    std::string err_msg = "xml data missing header";
    return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
  }
  std::string rev_major;
  std::string rev_minor;
  std::string database_name;
  std::string version;
  std::string date;
  double north = 0.0;
  double south = 0.0;
  double west = 0.0;
  double east = 0.0;
  std::string vendor;
  int checker =
      UtilXmlParser::QueryStringAttribute(*header_node, "revMajor", &rev_major);
  checker +=
      UtilXmlParser::QueryStringAttribute(*header_node, "revMinor", &rev_minor);
  checker +=
      UtilXmlParser::QueryStringAttribute(*header_node, "name", &database_name);
  checker +=
      UtilXmlParser::QueryStringAttribute(*header_node, "version", &version);
  checker += UtilXmlParser::QueryStringAttribute(*header_node, "date", &date);
  checker += header_node->QueryDoubleAttribute("north", &north);
  checker += header_node->QueryDoubleAttribute("south", &south);
  checker += header_node->QueryDoubleAttribute("east", &east);
  checker += header_node->QueryDoubleAttribute("west", &west);
  checker +=
      UtilXmlParser::QueryStringAttribute(*header_node, "vendor", &vendor);

  if (checker != tinyxml2::XML_SUCCESS) {
    std::string err_msg = "Error parsing header attributes";
    return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
  }

  auto geo_reference_node = header_node->FirstChildElement("geoReference");
  if (!geo_reference_node) {
    std::string err_msg = "Error parsing header geoReoference attributes";
    return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
  }
  auto geo_text = geo_reference_node->FirstChild()->ToText();
  if (!geo_text) {
    std::string err_msg = "Error parsing header geoReoference text";
    return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
  }

  // coordinate frame
  std::string zone_id;
  std::string from_coordinate = geo_text->Value();
  auto projection_node = header_node->FirstChildElement("projection");
  if (projection_node != nullptr) {
    auto utm_node = projection_node->FirstChildElement("utm");
    if (!utm_node) {
      std::string err_msg = "Error parsing header utm node";
      return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
    }
    checker =
        UtilXmlParser::QueryStringAttribute(*utm_node, "zoneID", &zone_id);
    if (checker != tinyxml2::XML_SUCCESS) {
      std::string err_msg = "Error parsing utm zone id attributes";
      return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
    }
  } else {
    int eastZone = GetLongZone(east);
    int westZone = GetLongZone(west);
    if (eastZone != westZone) {
      std::string err_msg = "unsupport data in more than one zones";
      return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
    }
    zone_id = std::to_string(westZone);
  }

  std::string to_coordinate = "+proj=utm +zone=" + zone_id +
                              " +ellps=WGS84 +datum=WGS84 +units=m +no_defs";
  CoordinateConvertTool::GetInstance()->SetConvertParam(from_coordinate,
                                                        to_coordinate);

  header->set_version(version);
  header->set_date(date);
  header->mutable_projection()->set_proj(to_coordinate);
  header->set_district(database_name);
  header->set_rev_major(rev_major);
  header->set_rev_minor(rev_minor);
  header->set_left(west);
  header->set_right(east);
  header->set_top(north);
  header->set_bottom(south);
  header->set_vendor(vendor);

  return Status::OK();
}

}  // namespace adapter
}  // namespace hdmap
}  // namespace apollo
