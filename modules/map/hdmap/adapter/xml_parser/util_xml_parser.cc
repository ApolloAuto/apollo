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

#include <sstream>
#include <iomanip>
#include <algorithm>
#include <limits>
#include <vector>
#include <string>
#include "modules/map/hdmap/adapter/coordinate_convert_tool.h"
#include "glog/logging.h"

namespace apollo {
namespace hdmap  {
namespace adapter {

Status UtilXmlParser::parse_geometry(const tinyxml2::XMLElement& xml_node,
                                    PbCurveSegment* curve_segment) {
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

        wgs84_to_utm(ptx, pty, ptz, &output_x, &output_y, &output_z);

        curve_segment->mutable_start_position()->set_x(output_x);
        curve_segment->mutable_start_position()->set_y(output_y);
        // TODO(liuyang23): add heading
        // curve_segment->set_heading(hdg);
        curve_segment->set_length(length);
    }

    const auto sub_node = xml_node.FirstChildElement("pointSet");
    if (sub_node) {
        PbLineSegment* line_segment = curve_segment->mutable_line_segment();
        RETURN_IF_ERROR(parse_point_set(*sub_node, line_segment));
        return Status::OK();
    }

    std::string err_msg = "Error geometry object";
    return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
}

Status UtilXmlParser::parse_point_set(const tinyxml2::XMLElement& xml_node,
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

        wgs84_to_utm(ptx, pty, ptz, &output_x, &output_y, &output_z);

        pt->set_x(output_x);
        pt->set_y(output_y);

        sub_node = sub_node->NextSiblingElement("point");
    }

    return Status::OK();
}

Status UtilXmlParser::parse_outline(const tinyxml2::XMLElement& xml_node,
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

        wgs84_to_utm(ptx, pty, ptz, &output_x, &output_y, &output_z);

        pt->set_x(output_x);
        pt->set_y(output_y);
        pt->set_z(output_z);

        sub_node = sub_node->NextSiblingElement("cornerGlobal");
    }

    return Status::OK();
}

Status UtilXmlParser::parse_point(const tinyxml2::XMLElement& xml_node,
                                PbPoint3D* pt) {
    const auto sub_node = xml_node.FirstChildElement("centerPoint");
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

    wgs84_to_utm(ptx, pty, ptz, &output_x, &output_y, &output_z);

    pt->set_x(output_x);
    pt->set_y(output_y);
    pt->set_z(output_z);

    return Status::OK();
}

std::vector<std::string> UtilXmlParser::split(const std::string str,
                                                const std::string pattern) {
    std::string::size_type pos;
    std::vector<std::string> result;
    std::string tmp_str = str + pattern;
    size_t size = tmp_str.size();

    for (size_t i = 0; i < size; i++) {
        pos = tmp_str.find(pattern, i);
        if (pos < size) {
            std::string s = tmp_str.substr(i, pos-i);
            result.push_back(s);
            i = pos + pattern.size() - 1;
        }
    }
    return result;
}

std::string UtilXmlParser::get_road_id(const std::string& lane_id) {
        std::string tmp_lane_id = lane_id;
        auto split_ids = split(tmp_lane_id, "_");
        assert(split_ids.size() > 0);
        return split_ids[0];
}

std::string UtilXmlParser::get_lane_id(const std::string& lane_id) {
    std::string tmp_lane_id = lane_id;
    auto split_ids = split(tmp_lane_id, "_");
    assert(split_ids.size() > 0);
    return split_ids.back();
}

std::string UtilXmlParser::get_traffic_light_sub_signal_id(
                                                const std::string& id) {
        std::string tmp_sub_signal_id = id;
        auto split_ids = split(tmp_sub_signal_id, "_");
        assert(split_ids.size() == 2);
        return split_ids[1];
}

std::string UtilXmlParser::create_lane_id(const std::string& road_id,
                                        const std::string& section_id,
                                        const int lane_id) {
    return road_id + "_" + std::to_string(lane_id);
}

void UtilXmlParser::to_upper(std::string* s) {
    CHECK_NOTNULL(s);
    std::transform(s->begin(), s->end(), s->begin(),
                   [](unsigned char c) { return std::toupper(c); });
}

void UtilXmlParser::utm_to_wgs84(const double x, const double y, const double z,
                        double* output_x, double* output_y, double* output_z) {
    _x_min = std::min(_x_min, x);
    _x_max = std::max(_x_max, x);
    _y_min = std::min(_y_min, y);
    _y_max = std::max(_y_max, y);
    CoordinateConvertTool::get_instance()->coordiate_convert(x, y, z,
                                                output_x, output_y, output_z);
}

void UtilXmlParser::wgs84_to_utm(const double x, const double y, const double z,
                        double* output_x, double* output_y, double* output_z) {
    CoordinateConvertTool::get_instance()->coordiate_convert(x, y, z,
                                                output_x, output_y, output_z);
}

double UtilXmlParser::_x_min = std::numeric_limits<double>::max();
double UtilXmlParser::_x_max = std::numeric_limits<double>::min();
double UtilXmlParser::_y_min = std::numeric_limits<double>::max();
double UtilXmlParser::_y_max = std::numeric_limits<double>::min();

void UtilXmlParser::get_map_boundary(double* x_min, double* x_max,
                                                double* y_min, double* y_max) {
    double z_max = 0.0;
    double z_min = 0.0;
    utm_to_wgs84(_x_min, _y_min, 0, x_min, y_min, &z_min);

    utm_to_wgs84(_x_max, _y_max, 0, x_max, y_max, &z_max);
}

double UtilXmlParser::curve_length(PbCurve* curve) {
    double length = 0.0;
    for (int i = 0; i < curve->segment_size(); ++i) {
        length += curve->segment(i).length();
    }

    return length;
}

tinyxml2::XMLError UtilXmlParser::query_string_attribute(
                                        const tinyxml2::XMLElement& xml_node,
                                        const std::string& name,
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
