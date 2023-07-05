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
#include "modules/map/hdmap/adapter/xml_parser/util_xml_parser.h"

#include <algorithm>

#include "cyber/common/log.h"
#include "modules/map/hdmap/adapter/xml_parser/coordinate_convert_tool.h"

namespace apollo {
namespace hdmap {
namespace adapter {

Status UtilXmlParser::ParseCurve(const tinyxml2::XMLElement& xml_node,
                                 PbCurve* curve) {
  CHECK_NOTNULL(curve);

  const tinyxml2::XMLElement* sub_node = xml_node.FirstChildElement("geometry");
  while (sub_node) {
    PbCurveSegment* curve_segment = curve->add_segment();
    RETURN_IF_ERROR(UtilXmlParser::ParseGeometry(*sub_node, curve_segment));
    sub_node = sub_node->NextSiblingElement("geometry");
  }

  return Status::OK();
}

Status UtilXmlParser::ParseGeometry(const tinyxml2::XMLElement& xml_node,
                                    PbCurveSegment* curve_segment) {
  CHECK_NOTNULL(curve_segment);

  // Read geometry attributes
  double s = 0.0;
  double ptx = 0.0;
  double pty = 0.0;
  double ptz = 0.0;
  double length = 0.0;

  int checker = tinyxml2::XML_SUCCESS;

  checker += xml_node.QueryDoubleAttribute("sOffset", &s);
  checker += xml_node.QueryDoubleAttribute("x", &ptx);
  checker += xml_node.QueryDoubleAttribute("y", &pty);
  checker += xml_node.QueryDoubleAttribute("length", &length);

  if (checker == tinyxml2::XML_SUCCESS) {
    curve_segment->set_s(s);

    double output_x = 0.0;
    double output_y = 0.0;
    double output_z = 0.0;

    WGS84ToUTM(ptx, pty, ptz, &output_x, &output_y, &output_z);

    curve_segment->mutable_start_position()->set_x(output_x);
    curve_segment->mutable_start_position()->set_y(output_y);
    curve_segment->set_length(length);
  }

  const auto sub_node = xml_node.FirstChildElement("pointSet");
  if (sub_node) {
    PbLineSegment* line_segment = curve_segment->mutable_line_segment();
    RETURN_IF_ERROR(ParsePointSet(*sub_node, line_segment));
    return Status::OK();
  }

  std::string err_msg = "Error geometry object";
  return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
}

Status UtilXmlParser::ParsePointSet(const tinyxml2::XMLElement& xml_node,
                                    PbLineSegment* line_segment) {
  const tinyxml2::XMLElement* sub_node = xml_node.FirstChildElement("point");
  while (sub_node) {
    double ptx = 0.0;
    double pty = 0.0;
    double ptz = 0.0;
    int checker = tinyxml2::XML_SUCCESS;
    checker += sub_node->QueryDoubleAttribute("x", &ptx);
    checker += sub_node->QueryDoubleAttribute("y", &pty);

    if (checker != tinyxml2::XML_SUCCESS) {
      std::string err_msg = "Error parsing geometry point attributes";
      return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
    }

    PbPoint3D* pt = line_segment->add_point();
    double output_x = 0.0;
    double output_y = 0.0;
    double output_z = 0.0;
    WGS84ToUTM(ptx, pty, ptz, &output_x, &output_y, &output_z);
    pt->set_x(output_x);
    pt->set_y(output_y);

    sub_node = sub_node->NextSiblingElement("point");
  }

  return Status::OK();
}

Status UtilXmlParser::ParseOutline(const tinyxml2::XMLElement& xml_node,
                                   PbPolygon* polygon) {
  const tinyxml2::XMLElement* sub_node =
      xml_node.FirstChildElement("cornerGlobal");
  while (sub_node) {
    double ptx = 0.0;
    double pty = 0.0;
    double ptz = 0.0;
    int checker = tinyxml2::XML_SUCCESS;
    checker += sub_node->QueryDoubleAttribute("x", &ptx);
    checker += sub_node->QueryDoubleAttribute("y", &pty);
    checker += sub_node->QueryDoubleAttribute("z", &ptz);

    if (checker != tinyxml2::XML_SUCCESS) {
      std::string err_msg = "Error parsing cornerGlobal point attributes";
      return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
    }

    PbPoint3D* pt = polygon->add_point();
    double output_x = 0.0;
    double output_y = 0.0;
    double output_z = 0.0;
    WGS84ToUTM(ptx, pty, ptz, &output_x, &output_y, &output_z);
    pt->set_x(output_x);
    pt->set_y(output_y);
    pt->set_z(output_z);

    sub_node = sub_node->NextSiblingElement("cornerGlobal");
  }

  return Status::OK();
}

Status UtilXmlParser::ParsePoint(const tinyxml2::XMLElement& xml_node,
                                 PbPoint3D* pt) {
  CHECK_NOTNULL(pt);

  const auto sub_node = xml_node.FirstChildElement("centerPoint");
  ACHECK(sub_node != nullptr);
  int checker = tinyxml2::XML_SUCCESS;
  double ptx = 0.0;
  double pty = 0.0;
  double ptz = 0.0;
  checker += sub_node->QueryDoubleAttribute("x", &ptx);
  checker += sub_node->QueryDoubleAttribute("y", &pty);
  checker += sub_node->QueryDoubleAttribute("z", &ptz);

  if (checker != tinyxml2::XML_SUCCESS) {
    std::string err_msg = "Error parse point attributes";
    return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
  }

  double output_x = 0.0;
  double output_y = 0.0;
  double output_z = 0.0;
  WGS84ToUTM(ptx, pty, ptz, &output_x, &output_y, &output_z);
  pt->set_x(output_x);
  pt->set_y(output_y);
  pt->set_z(output_z);

  return Status::OK();
}

std::string UtilXmlParser::ToUpper(const std::string& s) {
  std::string value = s;
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char c) { return std::toupper(c); });

  return value;
}

void UtilXmlParser::WGS84ToUTM(const double x, const double y, const double z,
                               double* output_x, double* output_y,
                               double* output_z) {
  CoordinateConvertTool::GetInstance()->CoordiateConvert(x, y, z, output_x,
                                                         output_y, output_z);
}

double UtilXmlParser::CurveLength(const PbCurve& curve) {
  double length = 0.0;
  for (int i = 0; i < curve.segment_size(); ++i) {
    length += curve.segment(i).length();
  }

  return length;
}

tinyxml2::XMLError UtilXmlParser::QueryStringAttribute(
    const tinyxml2::XMLElement& xml_node, const std::string& name,
    std::string* value) {
  CHECK_NOTNULL(value);
  const char* val = xml_node.Attribute(name.c_str());
  if (val == nullptr) {
    return tinyxml2::XML_NO_ATTRIBUTE;
  }

  *value = val;
  return tinyxml2::XML_SUCCESS;
}

}  // namespace adapter
}  // namespace hdmap
}  // namespace apollo
