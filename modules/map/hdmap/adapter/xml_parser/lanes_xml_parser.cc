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
#include "modules/map/hdmap/adapter/xml_parser/lanes_xml_parser.h"

#include <algorithm>
#include <string>
#include <vector>
#include "modules/map/hdmap/adapter/xml_parser/util_xml_parser.h"
#include "glog/logging.h"

namespace apollo {
namespace hdmap {
namespace adapter {

Status LanesXmlParser::parse(const tinyxml2::XMLElement& xml_node, Road* road) {
    CHECK_NOTNULL(road);

    const auto lanes_node = xml_node.FirstChildElement("lanes");
    assert(lanes_node != nullptr);
    const tinyxml2::XMLElement* sub_node =
                                lanes_node->FirstChildElement("laneSection");
    assert(sub_node != nullptr);

    size_t section_cnt = 0;
    std::string road_id = road->id;
    while (sub_node) {
        std::string section_id = std::to_string(section_cnt);
        RETURN_IF_ERROR(parse_lane_section(*sub_node, road_id,
                                        section_id, &road->lanes));
        sub_node = sub_node->NextSiblingElement("laneSection");

        ++section_cnt;
    }

    return Status::OK();
}

Status LanesXmlParser::parse_lane_section(const tinyxml2::XMLElement& xml_node,
                                        const std::string& road_id,
                                        const std::string& section_id,
                                        std::vector<LaneInternal>* lanes) {
    // left
    const tinyxml2::XMLElement* sub_node = xml_node.FirstChildElement("left");
    if (sub_node) {
        sub_node = sub_node->FirstChildElement("lane");
        while (sub_node) {
            LaneInternal lane_internal;
            RETURN_IF_ERROR(parse_lane(*sub_node, road_id, section_id,
                            &lane_internal));
            *(lane_internal.lane.mutable_left_boundary()) =
                                            lane_internal.lane.right_boundary();
            lane_internal.lane.clear_right_boundary();

            if (lanes->size() > 0) {
                PbLane& left_neighbor_lane = lanes->back().lane;
                *(left_neighbor_lane.mutable_right_boundary()) =
                                            lane_internal.lane.left_boundary();
            }

            // LaneInternal lane_internal;
            // lane_internal.lane = lane;
            lanes->push_back(lane_internal);
            sub_node = sub_node->NextSiblingElement("lane");
        }
    }

    // center
    // reference lane
    // PbLane reference_lane;
    LaneInternal reference_lane_internal;
    sub_node = xml_node.FirstChildElement("center");
    assert(sub_node != nullptr);

    sub_node = sub_node->FirstChildElement("lane");
    assert(sub_node != nullptr);
    RETURN_IF_ERROR(parse_lane(*sub_node, road_id, section_id,
                    &reference_lane_internal));
    *(reference_lane_internal.lane.mutable_left_boundary()) =
                                reference_lane_internal.lane.right_boundary();
    if (lanes->size() > 0) {
        PbLane& left_neighbor_lane = lanes->back().lane;
        *(left_neighbor_lane.mutable_right_boundary()) =
                                reference_lane_internal.lane.left_boundary();
    }

    sub_node = xml_node.FirstChildElement("right");
    if (sub_node) {
        sub_node = sub_node->FirstChildElement("lane");
        PbLane* left_neighbor_lane = &reference_lane_internal.lane;
        while (sub_node) {
            // PbLane lane;
            LaneInternal lane_internal;
            RETURN_IF_ERROR(parse_lane(*sub_node, road_id, section_id,
                                        &lane_internal));

            *(lane_internal.lane.mutable_left_boundary()) =
                                        left_neighbor_lane->right_boundary();
            // lane_internal.lane = lane;
            lanes->push_back(lane_internal);
            left_neighbor_lane = &lanes->back().lane;
            sub_node = sub_node->NextSiblingElement("lane");
        }
    }

    return Status::OK();
}

Status LanesXmlParser::parse_lane(const tinyxml2::XMLElement& xml_node,
                            const std::string& road_id,
                            const std::string& section_id,
                            LaneInternal* lane_internal) {
    // static int read_lane_overlap_size = 0;
    // static int read_object_overlap_size = 0;
    // static int read_junction_overlap_size = 0;
    // static int read_signal_overlap_size = 0;

    PbLane* lane = &lane_internal->lane;
    // lane id
    int checker = tinyxml2::XML_SUCCESS;
    int id = 0;
    checker += xml_node.QueryIntAttribute("id", &id);
    if (checker != tinyxml2::XML_SUCCESS) {
        std::string err_msg = "Error parse lane id";
        return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
    }
    std::string lane_id = UtilXmlParser::create_lane_id(road_id, section_id,
                                                        id);
    lane->mutable_id()->set_id(lane_id);

    // lane type
    std::string lane_type;
    checker = UtilXmlParser::query_string_attribute(xml_node, "type",
                                                    &lane_type);
    if (checker != tinyxml2::XML_SUCCESS) {
        std::string err_msg = "Error parse lane type.";
        return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
    }
    PbLaneType pb_lane_type;
    Status success = to_pb_lane_type(lane_type, &pb_lane_type);
    if (!success.ok()) {
        std::string err_msg = "Error convert lane type to pb lane type.";
        return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
    }
    lane->set_type(pb_lane_type);

    // border
    const tinyxml2::XMLElement* sub_node = xml_node.FirstChildElement("border");
    if (sub_node) {
        PbLaneBoundary* lane_boundary = lane->mutable_right_boundary();
        success = parse_lane_border(*sub_node, lane_boundary->mutable_curve());
        if (!success.ok()) {
            std::string err_msg = "Error parse lane border";
            return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR,
                err_msg);
        }
        lane_boundary->set_length(UtilXmlParser::curve_length(
                                            lane_boundary->mutable_curve()));
    }

    // road mark
    sub_node = sub_node->FirstChildElement("borderType");
    while (sub_node) {
        PbLaneBoundary* lane_boundary = lane->mutable_right_boundary();
        PbLaneBoundaryType boundary_type;
        success = parse_lane_border_mark(*sub_node, &boundary_type);
        if (!success.ok()) {
            std::string err_msg = "Error parse lane border type";
            return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR,
                err_msg);
        }
        lane_boundary->set_type(boundary_type);
        sub_node = sub_node->NextSiblingElement("borderType");
    }

    // reference line
    if (id == 0) {
        return Status::OK();
    }

    // turn type
    std::string turn_type;
    checker = UtilXmlParser::query_string_attribute(xml_node, "turnType",
                                                    &turn_type);
    if (checker != tinyxml2::XML_SUCCESS) {
        std::string err_msg = "Error parse lane turn type.";
        return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
    }
    PbTurnType pb_turn_type;
    success = to_pb_turn_type(turn_type, &pb_turn_type);
    if (!success.ok()) {
        std::string err_msg = "Error convert turn type to pb turn type.";
        return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
    }
    lane->set_turn(pb_turn_type);

    // direction
    std::string direction;
    checker = UtilXmlParser::query_string_attribute(xml_node, "direction",
                                                    &direction);
    if (checker != tinyxml2::XML_SUCCESS) {
        std::string err_msg = "Error parse lane direction.";
        return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
    }
    PbLaneDirection pb_lane_direction;
    success = to_pb_dirction(direction, &pb_lane_direction);
    if (!success.ok()) {
        std::string err_msg = "Error convert direction to pb direction";
        return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
    }
    lane->set_direction(pb_lane_direction);

    // link
    sub_node = xml_node.FirstChildElement("link");
    if (sub_node) {
        parse_lane_link(*sub_node, road_id, section_id, lane);
    }

    // center curve
    sub_node = xml_node.FirstChildElement("centerLine");
    if (sub_node) {
        PbCurve* center_curve = lane->mutable_central_curve();
        success = parse_lane_center_curve(*sub_node, center_curve);
        if (!success.ok()) {
            std::string err_msg = "Error parse lane center curve";
            return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
        }

        lane->set_length(UtilXmlParser::curve_length(center_curve));
    }

    // speed
    double max_speed = 0.0;
    sub_node = xml_node.FirstChildElement("speed");
    if (sub_node) {
        checker = sub_node->QueryDoubleAttribute("max", &max_speed);
        if (checker != tinyxml2::XML_SUCCESS) {
            std::string err_msg = "Error parse lane speed";
            return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR,
                err_msg);
        }
        lane->set_speed_limit(max_speed);
    }

    // sample association
    sub_node = xml_node.FirstChildElement("sampleAssociates");
    sub_node = sub_node->FirstChildElement("sampleAssociate");
    while (sub_node) {
        double left_width = 0.0;
        double right_width = 0.0;
        double s = 0.0;
        checker = sub_node->QueryDoubleAttribute("sOffset", &s);
        checker += sub_node->QueryDoubleAttribute("leftWidth", &left_width);
        checker += sub_node->QueryDoubleAttribute("rightWidth", &right_width);
        if (checker != tinyxml2::XML_SUCCESS) {
            std::string err_msg = "Error parse lane sample associate attribute";
            return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR,
                err_msg);
        }

        auto left_sample = lane->add_left_sample();
        left_sample->set_s(s);
        left_sample->set_width(left_width);

        auto right_sample = lane->add_right_sample();
        right_sample->set_s(s);
        right_sample->set_width(right_width);

        sub_node = sub_node->NextSiblingElement("sampleAssociate");
    }

    // overlap object
    sub_node = xml_node.FirstChildElement("objectOverlapGroup");
    if (sub_node) {
        sub_node = sub_node->FirstChildElement("objectReference");
        while (sub_node) {
            // read_object_overlap_size++;
            std::string object_id;
            double start_s = 0.0;
            double end_s = 0.0;
            int checker = UtilXmlParser::query_string_attribute(*sub_node,
                                                            "id", &object_id);
            checker += sub_node->QueryDoubleAttribute("startOffset", &start_s);
            checker += sub_node->QueryDoubleAttribute("endOffset", &end_s);
            if (checker != tinyxml2::XML_SUCCESS) {
                std::string err_msg = "Error parse lane object overlap";
                return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR,
                        err_msg);
            }

            OverlapWithLane overlap_with_lane;

            overlap_with_lane.object_id = object_id;
            overlap_with_lane.start_s = start_s;
            overlap_with_lane.end_s = end_s;
            lane_internal->overlap_objects.push_back(overlap_with_lane);

            sub_node = sub_node->NextSiblingElement("objectReference");
        }
    }

    // overlap signal
    sub_node = xml_node.FirstChildElement("signalOverlapGroup");
    if (sub_node) {
        sub_node = sub_node->FirstChildElement("signalReference");
        while (sub_node) {
            // read_signal_overlap_size++;
            std::string object_id;
            double start_s = 0.0;
            double end_s = 0.0;
            int checker = UtilXmlParser::query_string_attribute(*sub_node,
                                                            "id", &object_id);
            checker += sub_node->QueryDoubleAttribute("startOffset", &start_s);
            checker += sub_node->QueryDoubleAttribute("endOffset", &end_s);
            if (checker != tinyxml2::XML_SUCCESS) {
                std::string err_msg = "Error parse lane signal overlap";
                return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR,
                        err_msg);
            }

            OverlapWithLane overlap_with_lane;

            overlap_with_lane.object_id = object_id;
            overlap_with_lane.start_s = start_s;
            overlap_with_lane.end_s = end_s;
            lane_internal->overlap_signals.push_back(overlap_with_lane);

            sub_node = sub_node->NextSiblingElement("signalReference");
        }
    }

    // overlap junction
    sub_node = xml_node.FirstChildElement("junctionOverlapGroup");
    if (sub_node) {
        sub_node = sub_node->FirstChildElement("junctionReference");
        while (sub_node) {
            // read_junction_overlap_size++;
            std::string object_id;
            double start_s = 0.0;
            double end_s = 0.0;
            int checker = UtilXmlParser::query_string_attribute(*sub_node,
                                                            "id", &object_id);
            checker += sub_node->QueryDoubleAttribute("startOffset", &start_s);
            checker += sub_node->QueryDoubleAttribute("endOffset", &end_s);
            if (checker != tinyxml2::XML_SUCCESS) {
                std::string err_msg = "Error parse lane junction overlap";
                return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR,
                        err_msg);
            }

            OverlapWithLane overlap_with_lane;

            overlap_with_lane.object_id = object_id;
            overlap_with_lane.start_s = start_s;
            overlap_with_lane.end_s = end_s;
            lane_internal->overlap_junctions.push_back(overlap_with_lane);

            sub_node = sub_node->NextSiblingElement("junctionReference");
        }
    }

    // overlap lane
    sub_node = xml_node.FirstChildElement("laneOverlapGroup");
    if (sub_node) {
        sub_node = sub_node->FirstChildElement("laneReference");
        while (sub_node) {
            // read_lane_overlap_size++;
            std::string road_id;
            std::string section_id;
            int lane_id;
            double start_s = 0.0;
            double end_s = 0.0;
            int checker = UtilXmlParser::query_string_attribute(*sub_node,
                                                        "roadId", &road_id);
            checker += UtilXmlParser::query_string_attribute(*sub_node,
                                                    "sectionId", &section_id);
            checker += sub_node->QueryIntAttribute("laneId", &lane_id);
            checker += sub_node->QueryDoubleAttribute("startOffset", &start_s);
            checker += sub_node->QueryDoubleAttribute("endOffset", &end_s);
            if (checker != tinyxml2::XML_SUCCESS) {
                std::string err_msg = "Error parse lane lane overlap";
                return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR,
                        err_msg);
            }

            OverlapWithLane overlap_with_lane;

            overlap_with_lane.object_id = UtilXmlParser::create_lane_id(road_id,
                                                        section_id, lane_id);
            overlap_with_lane.start_s = start_s;
            overlap_with_lane.end_s = end_s;
            lane_internal->overlap_lanes.push_back(overlap_with_lane);

            sub_node = sub_node->NextSiblingElement("laneReference");
        }
    }

    // std::cout << "read_object_overlap_size:" << read_object_overlap_size
    //     << " read_signal_overlap_size:" << read_signal_overlap_size
    //     << " read_junction_overlap_size:" << read_junction_overlap_size
    //     << " read_lane_overlap_size:" << read_lane_overlap_size
    //     << std::endl;

    return Status::OK();
}

Status LanesXmlParser::to_pb_lane_type(const std::string& type,
                                    PbLaneType* lane_type) {
    CHECK_NOTNULL(lane_type);

    std::string upper_str(type);
    UtilXmlParser::to_upper(&upper_str);

    if (upper_str == "NONE") {
        *lane_type = ::apollo::hdmap::Lane::NONE;
    } else if (upper_str == "DRIVING") {
        *lane_type = ::apollo::hdmap::Lane::CITY_DRIVING;
    } else if (upper_str == "BIKING") {
        *lane_type = ::apollo::hdmap::Lane::BIKING;
    } else if (upper_str == "PARKING") {
        *lane_type = ::apollo::hdmap::Lane::PARKING;
    } else {
        std::string err_msg = "Error or unsupport lane type:" + type;
        return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
    }
    return Status::OK();
}

Status LanesXmlParser::to_pb_turn_type(const std::string& type,
                                    PbTurnType* pb_turn_type) {
    CHECK_NOTNULL(pb_turn_type);

    std::string upper_str(type);
    UtilXmlParser::to_upper(&upper_str);

    if (upper_str == "NOTURN") {
        *pb_turn_type = ::apollo::hdmap::Lane::NO_TURN;
    } else if (upper_str == "LEFTTURN") {
        *pb_turn_type = ::apollo::hdmap::Lane::LEFT_TURN;
    } else if (upper_str == "RIGHTTURN") {
        *pb_turn_type = ::apollo::hdmap::Lane::RIGHT_TURN;
    } else if (upper_str == "UTURN") {
        *pb_turn_type = ::apollo::hdmap::Lane::U_TURN;
    } else {
        std::string err_msg = "Error or unsupport turn type:" + type;
        return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
    }
    return Status::OK();
}

Status LanesXmlParser::to_pb_dirction(const std::string& type,
                                    PbLaneDirection* pb_direction) {
    CHECK_NOTNULL(pb_direction);

    std::string upper_str(type);
    UtilXmlParser::to_upper(&upper_str);

    if (upper_str == "FORWARD") {
        *pb_direction = ::apollo::hdmap::Lane::FORWARD;
    } else if (upper_str == "BACKWARD") {
        *pb_direction = ::apollo::hdmap::Lane::BACKWARD;
    } else if (upper_str == "BIDIRECTION") {
        *pb_direction = ::apollo::hdmap::Lane::BIDIRECTION;
    } else {
        std::string err_msg = "Error or unsupport dirction:" + type;
        return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
    }
    return Status::OK();
}

void LanesXmlParser::parse_lane_link(const tinyxml2::XMLElement& xml_node,
                                    const std::string& road_id,
                                    const std::string& section_id,
                                    PbLane* lane) {
    int predecessor = 0;
    const tinyxml2::XMLElement* sub_node =
                                    xml_node.FirstChildElement("predecessor");
    if (sub_node) {
        int checker = sub_node->QueryIntAttribute("id", &predecessor);
        if (checker == tinyxml2::XML_SUCCESS) {
            PbID* lane_id = lane->add_predecessor_id();
            std::string str_lane_id = UtilXmlParser::create_lane_id(
                            road_id, section_id, predecessor);
            lane_id->set_id(str_lane_id);
        }
    }

    int successor = 0;
    sub_node = xml_node.FirstChildElement("successor");
    if (sub_node) {
        int checker = sub_node->QueryIntAttribute("id", &successor);
        if (checker == tinyxml2::XML_SUCCESS) {
            PbID* lane_id = lane->add_successor_id();
            std::string str_lane_id = UtilXmlParser::create_lane_id(
                        road_id, section_id, successor);
            lane_id->set_id(str_lane_id);
        }
    }

    sub_node = xml_node.FirstChildElement("neighbor");
    if (sub_node) {
        int id = 0;
        std::string side;
        std::string direction;
        int checker = sub_node->QueryIntAttribute("id", &id);
        checker += UtilXmlParser::query_string_attribute(*sub_node,
                                                            "side", &side);
        checker += UtilXmlParser::query_string_attribute(*sub_node,
                                                    "direction", &direction);
        if (checker == tinyxml2::XML_SUCCESS) {
            std::string neighbor_id = UtilXmlParser::create_lane_id(
                        road_id, section_id, id);
            if (side == "left") {
                lane->add_left_neighbor_forward_lane_id()->set_id(neighbor_id);
            } else if (side == "right") {
                lane->add_right_neighbor_forward_lane_id()->set_id(neighbor_id);
            } else {
                assert(0);
            }
        }
    }
}

Status LanesXmlParser::parse_lane_center_curve(
                                        const tinyxml2::XMLElement& xml_node,
                                        PbCurve* center_curve) {
    const tinyxml2::XMLElement* sub_node
                                    = xml_node.FirstChildElement("geometry");
    if (!sub_node) {
        std::string err_msg = "Error parse lane center curve geometry.";
        return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
    }

    while (sub_node) {
        PbCurveSegment* curve_segment = center_curve->add_segment();
        RETURN_IF_ERROR(UtilXmlParser::parse_geometry(*sub_node,
                                                curve_segment));
        sub_node = sub_node->NextSiblingElement("geometry");
    }

    return Status::OK();
}

Status LanesXmlParser::parse_lane_border(const tinyxml2::XMLElement& xml_node,
                                      PbCurve* lane_border) {
    const tinyxml2::XMLElement* sub_node
                                    = xml_node.FirstChildElement("geometry");
    if (!sub_node) {
        std::string err_msg = "Error parse lane border geometry.";
        return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
    }

    while (sub_node) {
        PbCurveSegment* curve_segment = lane_border->add_segment();
        RETURN_IF_ERROR(UtilXmlParser::parse_geometry(*sub_node,
                                                    curve_segment));
        sub_node = sub_node->NextSiblingElement("geometry");
    }

    return Status::OK();
}

Status LanesXmlParser::parse_lane_speed(const tinyxml2::XMLElement& xml_node,
                                     PbLane *lane) {
    // double s_offset = 0;
    double max_speed = 0;

    int checker = tinyxml2::XML_SUCCESS;
    checker += xml_node.QueryDoubleAttribute("max", &max_speed);
    if (checker != tinyxml2::XML_SUCCESS) {
        std::string err_msg = "Error parse lane speed";
        return  Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
    }

    lane->set_speed_limit(max_speed);

    return Status::OK();
}

Status LanesXmlParser::parse_lane_border_mark(
                                        const tinyxml2::XMLElement& xml_node,
                                        PbLaneBoundaryType* boundary_type) {
    CHECK_NOTNULL(boundary_type);

    std::string type;
    std::string color;

    int checker = UtilXmlParser::query_string_attribute(xml_node,
                                                        "type", &type);
    checker += UtilXmlParser::query_string_attribute(xml_node,
                                                        "color", &color);
    if (checker != tinyxml2::XML_SUCCESS) {
        std::string err_msg = "Error to parse lane border mark";
        return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
    }

    // PbLaneBoundaryType boundary_type;
    Status success = to_pb_lane_mark_type(type, color, boundary_type);
    if (!success.ok()) {
        std::string err_msg = "fail to convert to pb lane border mark";
        return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
    }
    // pb_boundary->set_type(boundary_type);

    return Status::OK();
}

Status LanesXmlParser::to_pb_lane_mark_type(const std::string& type,
                                        const std::string& color,
                                        PbLaneBoundaryType* boundary_type) {
    CHECK_NOTNULL(boundary_type);

    std::string upper_type(type);
    UtilXmlParser::to_upper(&upper_type);

    std::string upper_color(color);
    UtilXmlParser::to_upper(&upper_color);

    if (upper_type == "CURB") {
        *boundary_type = ::apollo::hdmap::LaneBoundary::CURB;
        return Status::OK();
    }

    if (upper_type == "NONE") {
        *boundary_type = ::apollo::hdmap::LaneBoundary::UNKNOWN;
        return Status::OK();
    }

    if (upper_color == "YELLOW") {
        if (upper_type == "SOLID") {
            *boundary_type = ::apollo::hdmap::LaneBoundary::SOLID_YELLOW;
        } else if (upper_type == "BROKEN") {
            *boundary_type = ::apollo::hdmap::LaneBoundary::DOTTED_YELLOW;
        } else {
            std::string err_msg = "Error or unsupport lane boundary type";
            return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR,
                err_msg);
        }
    } else if (upper_color == "WHITE") {
        if (upper_type == "SOLID") {
            *boundary_type = ::apollo::hdmap::LaneBoundary::SOLID_WHITE;
        } else if (upper_type == "BROKEN") {
            *boundary_type = ::apollo::hdmap::LaneBoundary::DOTTED_WHITE;
        } else {
            std::string err_msg = "Error or unsupport lane boundary type";
            return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR,
                err_msg);
        }
    } else {
        std::string err_msg = "Error or unsupport lane boundary color.";
        return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
    }

    return Status::OK();
}

}  // namespace adapter
}  // namespace hdmap
}  // namespace apollo
