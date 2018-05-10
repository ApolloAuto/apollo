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

namespace {
double ToMPS(double speed) { return speed * 1000.0 / 3600.0; }
bool IsReferenceLane(int lane_id) { return lane_id == 0; }
};

namespace apollo {
namespace hdmap {
namespace adapter {

Status LanesXmlParser::Parse(const tinyxml2::XMLElement& xml_node,
                             const std::string& road_id,
                             std::vector<RoadSectionInternal>* sections) {
  CHECK_NOTNULL(sections);
  const auto lanes_node = xml_node.FirstChildElement("lanes");
  CHECK_NOTNULL(lanes_node);
  const tinyxml2::XMLElement* sub_node =
      lanes_node->FirstChildElement("laneSection");
  CHECK_NOTNULL(sub_node);

  size_t section_cnt = 0;
  while (sub_node) {
    RoadSectionInternal section_internal;
    std::string section_id = std::to_string(++section_cnt);
    section_internal.id = section_id;
    section_internal.section.mutable_id()->set_id(section_id);
    RETURN_IF_ERROR(ParseLaneSection(*sub_node,
                                     &section_internal.lanes));
    RETURN_IF_ERROR(ParseSectionBoundary(
        *sub_node,
        section_internal.section.mutable_boundary()->mutable_outer_polygon()));
    sections->push_back(section_internal);

    sub_node = sub_node->NextSiblingElement("laneSection");
  }

  CHECK_GT(sections->size(), 0);

  return Status::OK();
}

Status LanesXmlParser::ParseSectionBoundary(
    const tinyxml2::XMLElement& xml_node, PbBoundaryPolygon* boundary) {
  CHECK_NOTNULL(boundary);

  auto boundaries_node = xml_node.FirstChildElement("boundaries");
  if (boundaries_node == nullptr) {
    std::string err_msg = "Error parse boundaries";
    return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
  }

  auto sub_node = boundaries_node->FirstChildElement("boundary");
  while (sub_node) {
    PbBoundaryEdge* boundary_edge = boundary->add_edge();
    RETURN_IF_ERROR(
        UtilXmlParser::ParseCurve(*sub_node, boundary_edge->mutable_curve()));
    std::string type;
    int checker = UtilXmlParser::QueryStringAttribute(*sub_node, "type", &type);
    if (checker != tinyxml2::XML_SUCCESS) {
      std::string err_msg = "Error parse boundary type";
      return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
    }
    PbBoundaryEdgeType boundary_type;
    RETURN_IF_ERROR(ToPbBoundaryType(type, &boundary_type));
    boundary_edge->set_type(boundary_type);

    sub_node = sub_node->NextSiblingElement("boundary");
  }

  return Status::OK();
}

Status LanesXmlParser::ToPbBoundaryType(const std::string& type,
                                        PbBoundaryEdgeType* boundary_type) {
  CHECK_NOTNULL(boundary_type);

  std::string upper_type = UtilXmlParser::ToUpper(type);

  if (upper_type == "LEFTBOUNDARY") {
    *boundary_type = hdmap::BoundaryEdge::LEFT_BOUNDARY;
  } else if (upper_type == "RIGHTBOUNDARY") {
    *boundary_type = hdmap::BoundaryEdge::RIGHT_BOUNDARY;
  } else {
    *boundary_type = hdmap::BoundaryEdge::NORMAL;
  }

  return Status::OK();
}

Status LanesXmlParser::ParseLaneSection(const tinyxml2::XMLElement& xml_node,
                                        std::vector<LaneInternal>* lanes) {
  CHECK_NOTNULL(lanes);

  // left
  const tinyxml2::XMLElement* sub_node = xml_node.FirstChildElement("left");
  if (sub_node) {
    sub_node = sub_node->FirstChildElement("lane");
    while (sub_node) {
      LaneInternal lane_internal;
      RETURN_IF_ERROR(ParseLane(*sub_node, &lane_internal));
      *(lane_internal.lane.mutable_left_boundary()) =
          lane_internal.lane.right_boundary();
      lane_internal.lane.clear_right_boundary();
      if (lanes->size() > 0) {
        PbLane& left_neighbor_lane = lanes->back().lane;
        *(left_neighbor_lane.mutable_right_boundary()) =
            lane_internal.lane.left_boundary();
      }
      lanes->push_back(lane_internal);
      sub_node = sub_node->NextSiblingElement("lane");
    }
  }

  // center
  LaneInternal reference_lane_internal;
  sub_node = xml_node.FirstChildElement("center");
  CHECK_NOTNULL(sub_node);
  sub_node = sub_node->FirstChildElement("lane");
  CHECK_NOTNULL(sub_node);
  RETURN_IF_ERROR(ParseLane(*sub_node, &reference_lane_internal));
  *(reference_lane_internal.lane.mutable_left_boundary()) =
      reference_lane_internal.lane.right_boundary();
  if (lanes->size() > 0) {
    PbLane& left_neighbor_lane = lanes->back().lane;
    *(left_neighbor_lane.mutable_right_boundary()) =
        reference_lane_internal.lane.left_boundary();
  }

  // right
  sub_node = xml_node.FirstChildElement("right");
  if (sub_node) {
    sub_node = sub_node->FirstChildElement("lane");
    PbLane* left_neighbor_lane = &reference_lane_internal.lane;
    while (sub_node) {
      // PbLane lane
      LaneInternal lane_internal;
      RETURN_IF_ERROR(ParseLane(*sub_node, &lane_internal));
      *(lane_internal.lane.mutable_left_boundary()) =
          left_neighbor_lane->right_boundary();
      lanes->push_back(lane_internal);
      left_neighbor_lane = &lanes->back().lane;
      sub_node = sub_node->NextSiblingElement("lane");
    }
  }
  return Status::OK();
}

Status LanesXmlParser::ParseLane(const tinyxml2::XMLElement& xml_node,
                                 LaneInternal* lane_internal) {
  CHECK_NOTNULL(lane_internal);

  PbLane* lane = &lane_internal->lane;
  // lane id
  int id = 0;
  int checker = xml_node.QueryIntAttribute("id", &id);
  if (checker != tinyxml2::XML_SUCCESS) {
    std::string err_msg = "Error parse lane id";
    return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
  }
  std::string lane_id;
  checker = UtilXmlParser::QueryStringAttribute(xml_node, "uid", &lane_id);
  if (checker != tinyxml2::XML_SUCCESS) {
    std::string err_msg = "Error parse lane uid";
    return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
  }
  lane->mutable_id()->set_id(lane_id);

  // lane type
  std::string lane_type;
  checker = UtilXmlParser::QueryStringAttribute(xml_node, "type", &lane_type);
  if (checker != tinyxml2::XML_SUCCESS) {
    std::string err_msg = "Error parse lane type.";
    return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
  }
  PbLaneType pb_lane_type;
  Status success = ToPbLaneType(lane_type, &pb_lane_type);
  if (!success.ok()) {
    std::string err_msg = "Error convert lane type to pb lane type.";
    return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
  }
  lane->set_type(pb_lane_type);

  // border
  const tinyxml2::XMLElement* sub_node = xml_node.FirstChildElement("border");
  if (sub_node) {
    PbLaneBoundary* lane_boundary = lane->mutable_right_boundary();
    CHECK(lane_boundary != nullptr);
    success =
        UtilXmlParser::ParseCurve(*sub_node, lane_boundary->mutable_curve());
    if (!success.ok()) {
      std::string err_msg = "Error parse lane border";
      return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
    }
    lane_boundary->set_length(
        UtilXmlParser::CurveLength(*lane_boundary->mutable_curve()));

    bool is_virtual = false;
    std::string virtual_border = "FALSE";
    checker = UtilXmlParser::QueryStringAttribute(*sub_node, "virtual",
            &virtual_border);
    if (checker == tinyxml2::XML_SUCCESS) {
      if (virtual_border == "TRUE") {
        is_virtual = true;
      }
    }
    lane_boundary->set_virtual_(is_virtual);
  }

  // road mark
  if (sub_node) {
    sub_node = sub_node->FirstChildElement("borderType");
  }
  while (sub_node) {
    PbLaneBoundary* lane_boundary = lane->mutable_right_boundary();
    PbLaneBoundaryTypeType boundary_type;
    success = ParseLaneBorderMark(*sub_node, &boundary_type);
    if (!success.ok()) {
      std::string err_msg = "Error parse lane border type";
      return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
    }
    double s_offset = 0.0;
    checker = sub_node->QueryDoubleAttribute("sOffset", &s_offset);
    if (checker != tinyxml2::XML_SUCCESS) {
      std::string err_msg = "Error parse lane boundary type offset.";
      return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
    }
    auto lane_boundary_type = lane_boundary->add_boundary_type();
    lane_boundary_type->set_s(s_offset);
    lane_boundary_type->add_types(boundary_type);
    sub_node = sub_node->NextSiblingElement("borderType");
  }

  // reference line
  if (IsReferenceLane(id)) {
    return Status::OK();
  }

  // turn type
  std::string turn_type;
  checker =
      UtilXmlParser::QueryStringAttribute(xml_node, "turnType", &turn_type);
  if (checker != tinyxml2::XML_SUCCESS) {
    std::string err_msg = "Error parse lane turn type.";
    return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
  }
  PbTurnType pb_turn_type;
  success = ToPbTurnType(turn_type, &pb_turn_type);
  if (!success.ok()) {
    std::string err_msg = "Error convert turn type to pb turn type.";
    return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
  }
  lane->set_turn(pb_turn_type);

  // direction
  RETURN_IF_ERROR(ParseDirection(xml_node, lane));

  // link
  sub_node = xml_node.FirstChildElement("link");
  if (sub_node) {
    ParseLaneLink(*sub_node, lane);
  }

  // center curve
  RETURN_IF_ERROR(ParseCenterCurve(xml_node, lane));
  // speed
  RETURN_IF_ERROR(ParseSpeed(xml_node, lane));
  // sample association
  RETURN_IF_ERROR(ParseSampleAssociates(xml_node, lane));
  // road sample association
  RETURN_IF_ERROR(ParseRoadSampleAssociates(xml_node, lane));

  // overlap object
  ParseObjectOverlapGroup(xml_node, &lane_internal->overlap_objects);
  // overlap signal
  ParseSignalOverlapGroup(xml_node, &lane_internal->overlap_signals);
  // overlap junction
  ParseJunctionOverlapGroup(xml_node, &lane_internal->overlap_junctions);
  // overlap lane
  ParseLaneOverlapGroup(xml_node, &lane_internal->overlap_lanes);

  return Status::OK();
}

Status LanesXmlParser::ParseDirection(const tinyxml2::XMLElement& xml_node,
                                      PbLane* lane) {
  CHECK_NOTNULL(lane);
  std::string direction;
  int checker =
      UtilXmlParser::QueryStringAttribute(xml_node, "direction", &direction);
  if (checker != tinyxml2::XML_SUCCESS) {
    std::string err_msg = "Error parse lane direction.";
    return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
  }
  PbLaneDirection pb_lane_direction;
  Status success = ToPbDirection(direction, &pb_lane_direction);
  if (!success.ok()) {
    std::string err_msg = "Error convert direction to pb direction";
    return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
  }
  lane->set_direction(pb_lane_direction);

  return Status::OK();
}

Status LanesXmlParser::ParseCenterCurve(const tinyxml2::XMLElement& xml_node,
                                        PbLane* lane) {
  CHECK_NOTNULL(lane);
  auto sub_node = xml_node.FirstChildElement("centerLine");
  if (!sub_node) {
    std::string err_msg = "Error parse lane center curve";
    return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
  }

  PbCurve* center_curve = lane->mutable_central_curve();
  Status success = UtilXmlParser::ParseCurve(*sub_node, center_curve);
  if (!success.ok()) {
    std::string err_msg = "Error parse lane center curve";
    return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
  }

  lane->set_length(UtilXmlParser::CurveLength(*center_curve));

  return Status::OK();
}

Status LanesXmlParser::ParseSpeed(const tinyxml2::XMLElement& xml_node,
                                  PbLane* lane) {
  double max_speed = 0.0;
  auto sub_node = xml_node.FirstChildElement("speed");
  if (sub_node) {
    int checker = sub_node->QueryDoubleAttribute("max", &max_speed);
    if (checker != tinyxml2::XML_SUCCESS) {
      std::string err_msg = "Error parse lane speed attribute";
      return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
    }
    lane->set_speed_limit(ToMPS(max_speed));
  }

  return Status::OK();
}

Status LanesXmlParser::ParseSampleAssociates(
    const tinyxml2::XMLElement& xml_node, PbLane* lane) {
  CHECK_NOTNULL(lane);
  auto sub_node = xml_node.FirstChildElement("sampleAssociates");
  if (sub_node == nullptr) {
    std::string err_msg = "Error parse sample associates";
    return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
  }
  sub_node = sub_node->FirstChildElement("sampleAssociate");
  if (sub_node == nullptr) {
    std::string err_msg = "Error parse sample associate";
    return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
  }

  while (sub_node) {
    double left_width = 0.0;
    double right_width = 0.0;
    double s = 0.0;
    int checker = sub_node->QueryDoubleAttribute("sOffset", &s);
    checker += sub_node->QueryDoubleAttribute("leftWidth", &left_width);
    checker += sub_node->QueryDoubleAttribute("rightWidth", &right_width);
    if (checker != tinyxml2::XML_SUCCESS) {
      std::string err_msg = "Error parse lane sample associate attribute";
      return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
    }

    auto left_sample = lane->add_left_sample();
    left_sample->set_s(s);
    left_sample->set_width(left_width);

    auto right_sample = lane->add_right_sample();
    right_sample->set_s(s);
    right_sample->set_width(right_width);

    sub_node = sub_node->NextSiblingElement("sampleAssociate");
  }

  return Status::OK();
}

Status LanesXmlParser::ParseRoadSampleAssociates(
    const tinyxml2::XMLElement& xml_node, PbLane* lane) {
  CHECK_NOTNULL(lane);
  auto sub_node = xml_node.FirstChildElement("roadSampleAssociations");
  if (sub_node == nullptr) {
    std::string err_msg = "Error parse road sample associations";
    return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
  }
  sub_node = sub_node->FirstChildElement("sampleAssociation");
  if (sub_node == nullptr) {
    std::string err_msg = "Error parse road sample association";
    return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
  }

  while (sub_node) {
    double left_width = 0.0;
    double right_width = 0.0;
    double s = 0.0;
    int checker = sub_node->QueryDoubleAttribute("sOffset", &s);
    checker += sub_node->QueryDoubleAttribute("leftWidth", &left_width);
    checker += sub_node->QueryDoubleAttribute("rightWidth", &right_width);
    if (checker != tinyxml2::XML_SUCCESS) {
      std::string err_msg = "Error parse road sample association attribute";
      return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
    }

    auto left_road_sample = lane->add_left_road_sample();
    left_road_sample->set_s(s);
    left_road_sample->set_width(left_width);

    auto right_road_sample = lane->add_right_road_sample();
    right_road_sample->set_s(s);
    right_road_sample->set_width(right_width);

    sub_node = sub_node->NextSiblingElement("sampleAssociation");
  }

  return Status::OK();
}

Status LanesXmlParser::ParseObjectOverlapGroup(
    const tinyxml2::XMLElement& xml_node,
    std::vector<OverlapWithLane>* object_overlaps) {
  CHECK_NOTNULL(object_overlaps);

  auto object_overlap = xml_node.FirstChildElement("objectOverlapGroup");
  if (object_overlap) {
    auto sub_node = object_overlap->FirstChildElement("objectReference");
    while (sub_node) {
      std::string object_id;
      double start_s = 0.0;
      double end_s = 0.0;
      int checker =
          UtilXmlParser::QueryStringAttribute(*sub_node, "id", &object_id);
      checker += sub_node->QueryDoubleAttribute("startOffset", &start_s);
      checker += sub_node->QueryDoubleAttribute("endOffset", &end_s);
      if (checker != tinyxml2::XML_SUCCESS) {
        std::string err_msg = "Error parse lane object overlap";
        return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
      }

      bool is_merge = false;
      checker = sub_node->QueryBoolAttribute("isMerge", &is_merge);
      if (checker != tinyxml2::XML_SUCCESS) {
        is_merge = false;
      }

      OverlapWithLane overlap_with_lane;
      overlap_with_lane.object_id = object_id;
      overlap_with_lane.start_s = start_s;
      overlap_with_lane.end_s = end_s;
      overlap_with_lane.is_merge = is_merge;
      object_overlaps->push_back(overlap_with_lane);

      sub_node = sub_node->NextSiblingElement("objectReference");
    }
  }
  return Status::OK();
}

Status LanesXmlParser::ParseSignalOverlapGroup(
    const tinyxml2::XMLElement& xml_node,
    std::vector<OverlapWithLane>* signal_overlaps) {
  CHECK_NOTNULL(signal_overlaps);

  auto signal_overlap = xml_node.FirstChildElement("signalOverlapGroup");
  if (signal_overlap) {
    auto sub_node = signal_overlap->FirstChildElement("signalReference");
    while (sub_node) {
      std::string object_id;
      double start_s = 0.0;
      double end_s = 0.0;
      int checker =
          UtilXmlParser::QueryStringAttribute(*sub_node, "id", &object_id);
      checker += sub_node->QueryDoubleAttribute("startOffset", &start_s);
      checker += sub_node->QueryDoubleAttribute("endOffset", &end_s);
      if (checker != tinyxml2::XML_SUCCESS) {
        std::string err_msg = "Error parse lane signal overlap";
        return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
      }

      bool is_merge = false;
      checker = sub_node->QueryBoolAttribute("isMerge", &is_merge);
      if (checker != tinyxml2::XML_SUCCESS) {
        is_merge = false;
      }

      OverlapWithLane overlap_with_lane;
      overlap_with_lane.object_id = object_id;
      overlap_with_lane.start_s = start_s;
      overlap_with_lane.end_s = end_s;
      overlap_with_lane.is_merge = is_merge;

      signal_overlaps->push_back(overlap_with_lane);

      sub_node = sub_node->NextSiblingElement("signalReference");
    }
  }
  return Status::OK();
}

Status LanesXmlParser::ParseJunctionOverlapGroup(
    const tinyxml2::XMLElement& xml_node,
    std::vector<OverlapWithLane>* junction_overlaps) {
  CHECK_NOTNULL(junction_overlaps);

  auto overlap_group = xml_node.FirstChildElement("junctionOverlapGroup");
  if (overlap_group) {
    auto sub_node = overlap_group->FirstChildElement("junctionReference");
    while (sub_node) {
      std::string object_id;
      double start_s = 0.0;
      double end_s = 0.0;
      int checker =
          UtilXmlParser::QueryStringAttribute(*sub_node, "id", &object_id);
      checker += sub_node->QueryDoubleAttribute("startOffset", &start_s);
      checker += sub_node->QueryDoubleAttribute("endOffset", &end_s);
      if (checker != tinyxml2::XML_SUCCESS) {
        std::string err_msg = "Error parse lane junction overlap";
        return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
      }

      bool is_merge = false;
      checker = sub_node->QueryBoolAttribute("isMerge", &is_merge);
      if (checker != tinyxml2::XML_SUCCESS) {
        is_merge = false;
      }

      OverlapWithLane overlap_with_lane;
      overlap_with_lane.object_id = object_id;
      overlap_with_lane.start_s = start_s;
      overlap_with_lane.end_s = end_s;
      overlap_with_lane.is_merge = is_merge;

      junction_overlaps->push_back(overlap_with_lane);

      sub_node = sub_node->NextSiblingElement("junctionReference");
    }
  }
  return Status::OK();
}

Status LanesXmlParser::ParseLaneOverlapGroup(
    const tinyxml2::XMLElement& xml_node,
    std::vector<OverlapWithLane>* lane_overlaps) {
  CHECK_NOTNULL(lane_overlaps);

  auto overlap_node = xml_node.FirstChildElement("laneOverlapGroup");
  if (overlap_node) {
    auto sub_node = overlap_node->FirstChildElement("laneReference");
    while (sub_node) {
      std::string lane_id;
      double start_s = 0.0;
      double end_s = 0.0;
      int checker = UtilXmlParser::QueryStringAttribute(*sub_node, "id",
                                                   &lane_id);
      checker += sub_node->QueryDoubleAttribute("startOffset", &start_s);
      checker += sub_node->QueryDoubleAttribute("endOffset", &end_s);
      if (checker != tinyxml2::XML_SUCCESS) {
        std::string err_msg = "Error parse lane lane overlap";
        return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
      }

      bool is_merge = false;
      checker = sub_node->QueryBoolAttribute("isMerge", &is_merge);
      if (checker != tinyxml2::XML_SUCCESS) {
        is_merge = false;
      }

      OverlapWithLane overlap_with_lane;
      overlap_with_lane.object_id = lane_id;
      overlap_with_lane.start_s = start_s;
      overlap_with_lane.end_s = end_s;
      overlap_with_lane.is_merge = is_merge;

      lane_overlaps->push_back(overlap_with_lane);

      sub_node = sub_node->NextSiblingElement("laneReference");
    }
  }

  return Status::OK();
}

Status LanesXmlParser::ToPbLaneType(const std::string& type,
                                    PbLaneType* lane_type) {
  CHECK_NOTNULL(lane_type);

  std::string upper_str = UtilXmlParser::ToUpper(type);

  if (upper_str == "NONE") {
    *lane_type = hdmap::Lane::NONE;
  } else if (upper_str == "DRIVING") {
    *lane_type = hdmap::Lane::CITY_DRIVING;
  } else if (upper_str == "BIKING") {
    *lane_type = hdmap::Lane::BIKING;
  } else if (upper_str == "PARKING") {
    *lane_type = hdmap::Lane::PARKING;
  } else {
    std::string err_msg = "Error or unsupport lane type:" + type;
    return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
  }

  return Status::OK();
}

Status LanesXmlParser::ToPbTurnType(const std::string& type,
                                    PbTurnType* pb_turn_type) {
  CHECK_NOTNULL(pb_turn_type);

  std::string upper_str = UtilXmlParser::ToUpper(type);

  if (upper_str == "NOTURN") {
    *pb_turn_type = hdmap::Lane::NO_TURN;
  } else if (upper_str == "LEFTTURN") {
    *pb_turn_type = hdmap::Lane::LEFT_TURN;
  } else if (upper_str == "RIGHTTURN") {
    *pb_turn_type = hdmap::Lane::RIGHT_TURN;
  } else if (upper_str == "UTURN") {
    *pb_turn_type = hdmap::Lane::U_TURN;
  } else {
    std::string err_msg = "Error or unsupport turn type:" + type;
    return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
  }

  return Status::OK();
}

Status LanesXmlParser::ToPbDirection(const std::string& type,
                                     PbLaneDirection* pb_direction) {
  CHECK_NOTNULL(pb_direction);

  std::string upper_str = UtilXmlParser::ToUpper(type);

  if (upper_str == "FORWARD") {
    *pb_direction = hdmap::Lane::FORWARD;
  } else if (upper_str == "BACKWARD") {
    *pb_direction = hdmap::Lane::BACKWARD;
  } else if (upper_str == "BIDIRECTION") {
    *pb_direction = hdmap::Lane::BIDIRECTION;
  } else {
    std::string err_msg = "Error or unsupport dirction:" + type;
    return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
  }

  return Status::OK();
}

void LanesXmlParser::ParseLaneLink(const tinyxml2::XMLElement& xml_node,
                                   PbLane* lane) {
  CHECK_NOTNULL(lane);

  const tinyxml2::XMLElement* sub_node =
      xml_node.FirstChildElement("predecessor");
  while (sub_node) {
    std::string lane_id;
    int checker = UtilXmlParser::QueryStringAttribute(*sub_node, "id",
                                                    &lane_id);
    if (checker == tinyxml2::XML_SUCCESS) {
      PbID* pb_lane_id = lane->add_predecessor_id();
      pb_lane_id->set_id(lane_id);
    }
    sub_node = sub_node->NextSiblingElement("predecessor");
  }

  sub_node = xml_node.FirstChildElement("successor");
  while (sub_node) {
    std::string lane_id;
    int checker = UtilXmlParser::QueryStringAttribute(*sub_node, "id",
                                                    &lane_id);
    if (checker == tinyxml2::XML_SUCCESS) {
      PbID* pb_lane_id = lane->add_successor_id();
      pb_lane_id->set_id(lane_id);
    }
    sub_node = sub_node->NextSiblingElement("successor");
  }
  sub_node = xml_node.FirstChildElement("neighbor");
  while (sub_node) {
    std::string side;
    std::string direction;
    std::string lane_id;
    int checker = UtilXmlParser::QueryStringAttribute(*sub_node, "id",
                                                   &lane_id);
    checker += UtilXmlParser::QueryStringAttribute(*sub_node, "side", &side);
    checker +=
        UtilXmlParser::QueryStringAttribute(*sub_node, "direction", &direction);
    if (checker == tinyxml2::XML_SUCCESS) {
      if (side == "left") {
        lane->add_left_neighbor_forward_lane_id()->set_id(lane_id);
      } else if (side == "right") {
        lane->add_right_neighbor_forward_lane_id()->set_id(lane_id);
      }
    }
    sub_node = sub_node->NextSiblingElement("neighbor");
  }
}

Status LanesXmlParser::ParseLaneBorderMark(
    const tinyxml2::XMLElement& xml_node,
    PbLaneBoundaryTypeType* boundary_type) {
  CHECK_NOTNULL(boundary_type);

  std::string type;
  std::string color;

  int checker = UtilXmlParser::QueryStringAttribute(xml_node, "type", &type);
  checker += UtilXmlParser::QueryStringAttribute(xml_node, "color", &color);
  if (checker != tinyxml2::XML_SUCCESS) {
    std::string err_msg = "Error to parse lane border mark";
    return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
  }

  Status success = ToPbLaneMarkType(type, color, boundary_type);
  if (!success.ok()) {
    std::string err_msg = "fail to convert to pb lane border mark";
    return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
  }

  return Status::OK();
}

Status LanesXmlParser::ToPbLaneMarkType(const std::string& type,
                                        const std::string& color,
                                        PbLaneBoundaryTypeType* boundary_type) {
  CHECK_NOTNULL(boundary_type);

  std::string upper_type = UtilXmlParser::ToUpper(type);
  std::string upper_color = UtilXmlParser::ToUpper(color);

  if (upper_type == "CURB") {
    *boundary_type = hdmap::LaneBoundaryType::CURB;
    return Status::OK();
  }

  if (upper_type == "NONE") {
    *boundary_type = hdmap::LaneBoundaryType::UNKNOWN;
    return Status::OK();
  }

  if (upper_color == "YELLOW") {
    if (upper_type == "SOLID") {
      *boundary_type = hdmap::LaneBoundaryType::SOLID_YELLOW;
    } else if (upper_type == "BROKEN") {
      *boundary_type = hdmap::LaneBoundaryType::DOTTED_YELLOW;
    } else {
      std::string err_msg = "Error or unsupport lane boundary type";
      return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
    }
  } else if (upper_color == "WHITE") {
    if (upper_type == "SOLID") {
      *boundary_type = hdmap::LaneBoundaryType::SOLID_WHITE;
    } else if (upper_type == "BROKEN") {
      *boundary_type = hdmap::LaneBoundaryType::DOTTED_WHITE;
    } else {
      std::string err_msg = "Error or unsupport lane boundary type";
      return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
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
