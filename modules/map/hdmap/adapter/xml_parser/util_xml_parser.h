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
#ifndef MODULES_MAP_MAP_LOADER_ADAPTER_XML_PARSER_UTIL_XML_PARSER_H
#define MODULES_MAP_MAP_LOADER_ADAPTER_XML_PARSER_UTIL_XML_PARSER_H

#include <vector>
#include <string>
#include "modules/map/hdmap/adapter/xml_parser/common_define.h"
#include "tinyxml2.h"
#include "modules/map/hdmap/adapter/xml_parser/status.h"

namespace apollo {
namespace hdmap {
namespace adapter {

class UtilXmlParser {
 public:
  static Status ParseCurve(const tinyxml2::XMLElement& xml_node,
                            PbCurve* curve);
  static Status ParseGeometry(const tinyxml2::XMLElement& xml_node,
                              PbCurveSegment* curve_segment);
  static Status ParsePointSet(const tinyxml2::XMLElement& xml_node,
                              PbLineSegment* line_segment);
  static Status ParseOutline(const tinyxml2::XMLElement& xml_node,
                              PbPolygon* polygon);
  static Status ParsePoint(const tinyxml2::XMLElement& xml_node,
                              PbPoint3D* pt);

  static std::string CreateLaneId(const std::string& road_id,
                                  const std::string& section_id,
                                  const int lane_id);
  static std::string ToUpper(const std::string& s);

  static void WGS84ToUTM(const double x, const double y, const double z,
                      double* output_x, double* output_y, double* output_z);

  static double CurveLength(const PbCurve& curve);

  static tinyxml2::XMLError QueryStringAttribute(
                                      const tinyxml2::XMLElement& xml_node,
                                      const std::string& name,
                                      std::string* value);

 private:
  static double x_min_;
  static double x_max_;
  static double y_min_;
  static double y_max_;
};

int GetLongZone(double longitude);

}  // namespace adapter
}  // namespace hdmap
}  // namespace apollo

#endif  // MODULES_MAP_MAP_LOADER_ADAPTER_XML_PARSER_UTIL_XML_PARSER_H
