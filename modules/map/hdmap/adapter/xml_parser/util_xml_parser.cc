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
#include "modules/map/hdmap/adapter/coordinate_convert_tool.h"

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
  CHECK(sub_node != nullptr);
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
Status UtilXmlParser::ParseBoundaryPolygon(const tinyxml2::XMLElement& xml_node,
                                  PbBoundaryPolygon* boundary_polygon) {
  CHECK_NOTNULL(boundary_polygon);

  auto sub_node = xml_node.FirstChildElement("boundary_polygon");
  if (sub_node) {
    auto egde_node = sub_node->FirstChildElement("boundary_edge");
    while (egde_node) {
      PbBoundaryEdge* boundary_edge = boundary_polygon->add_edge();
      RETURN_IF_ERROR(ParseBoundaryEdge(*egde_node, boundary_edge));

      egde_node = egde_node->NextSiblingElement("boundary_edge");
    }

    sub_node = sub_node->NextSiblingElement("boundary_polygon");
  }
  return Status::OK();
}

Status UtilXmlParser::ParseBoundaryEdge(const tinyxml2::XMLElement& xml_node,
                                  PbBoundaryEdge* boundary_edge) {
  CHECK_NOTNULL(boundary_edge);

  std::string object_type;
  const auto curve_node = xml_node.FirstChildElement("curve");
  if (curve_node) {
    const auto geometry_node = curve_node->FirstChildElement("geometry");
    if (geometry_node) {
      PbCurve* curve = boundary_edge->mutable_curve();
      PbCurveSegment* curve_segment = curve->add_segment();
      CHECK(curve_segment != nullptr);
      RETURN_IF_ERROR(
        UtilXmlParser::ParseGeometry(*geometry_node, curve_segment));
    }
  }

  auto egdg_type_info_node = xml_node.FirstChildElement("edge_type_info");
  while  (egdg_type_info_node) {
    double start_s = 0;
    double end_s = 0;
    int checker =
           egdg_type_info_node->QueryDoubleAttribute("start_s", &start_s);
    checker += egdg_type_info_node->QueryDoubleAttribute("end_s", &end_s);
    if (checker != tinyxml2::XML_SUCCESS) {
      std::string err_msg = "Error parse start_s and end_s ";
      return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
    }
    auto edge_type_info = boundary_edge->add_edge_type_info();
    edge_type_info->set_start_s(start_s);
    edge_type_info->set_end_s(end_s);
    auto egdg_type_info_type_node =
            egdg_type_info_node->FirstChildElement("edge_type_info_type");
    while (egdg_type_info_type_node) {
      checker = UtilXmlParser::QueryStringAttribute(
            *egdg_type_info_type_node, "type", &object_type);
      if (checker != tinyxml2::XML_SUCCESS) {
        std::string err_msg = "Error parse object type.";
        return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
      }

      PbRoadBoundaryEdgeType pb_boundary_edge_type;
      ToBoundaryEdgeType(object_type, &pb_boundary_edge_type);
      edge_type_info->add_type(pb_boundary_edge_type);
      egdg_type_info_type_node =
          egdg_type_info_type_node->NextSiblingElement("edge_type_info_type");
    }

    egdg_type_info_node =
          egdg_type_info_node->NextSiblingElement("edge_type_info");
  }
  return Status::OK();
}

Status UtilXmlParser::ToBoundaryEdgeType(const std::string &edge_type,
                        PbRoadBoundaryEdgeType* pb_boundary_edge_type) {
  CHECK_NOTNULL(pb_boundary_edge_type);

  *pb_boundary_edge_type = apollo::hdmap::DEFAULT;
  if (edge_type == "DEFAULT") {
    *pb_boundary_edge_type = apollo::hdmap::DEFAULT;
  } else if (edge_type == "ROAD_SURFACE") {
    *pb_boundary_edge_type = apollo::hdmap::SURFACE;
  } else if (edge_type == "ROAD_CURB") {
    *pb_boundary_edge_type = apollo::hdmap::CURB;
  } else if (edge_type == "FENCE") {
    *pb_boundary_edge_type = apollo::hdmap::FENCE;
  } else if (edge_type == "WALL") {
    *pb_boundary_edge_type = apollo::hdmap::WALL;
  } else if (edge_type == "SUNKEN") {
    *pb_boundary_edge_type = apollo::hdmap::SUNKEN;
  } else if (edge_type == "PED_XING") {
    *pb_boundary_edge_type = apollo::hdmap::PED_XING;
  } else if (edge_type == "VIRTUAL") {
    *pb_boundary_edge_type = apollo::hdmap::VIRTUAL;
  }
  return Status::OK();
}

Status UtilXmlParser::ToEdgeType(const std::string &edge_type,
                                 PbBoundaryEdgeType* pb_edge_type) {
  CHECK_NOTNULL(pb_edge_type);

  *pb_edge_type = apollo::hdmap::BoundaryEdge_Type_UNKNOWN;
  if (edge_type == "UNKNOWN") {
    *pb_edge_type = apollo::hdmap::BoundaryEdge_Type_UNKNOWN;
  } else if (edge_type == "NORMAL") {
    *pb_edge_type = apollo::hdmap::BoundaryEdge_Type_NORMAL;
  } else if (edge_type == "LEFT_BOUNDARY") {
    *pb_edge_type = apollo::hdmap::BoundaryEdge_Type_LEFT_BOUNDARY;
  } else if (edge_type == "RIGHT_BOUNDARY") {
    *pb_edge_type = apollo::hdmap::BoundaryEdge_Type_RIGHT_BOUNDARY;
  }
  return Status::OK();
}

}  // namespace adapter
}  // namespace hdmap
}  // namespace apollo
