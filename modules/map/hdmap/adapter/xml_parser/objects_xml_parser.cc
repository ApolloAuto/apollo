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

#include "modules/map/hdmap/adapter/xml_parser/objects_xml_parser.h"

#include <string>

#include "modules/map/hdmap/adapter/xml_parser/util_xml_parser.h"

namespace apollo {
namespace hdmap {
namespace adapter {

Status ObjectsXmlParser::ParseCrosswalks(const tinyxml2::XMLElement& xml_node,
                                         std::vector<PbCrosswalk>* crosswalks) {
  CHECK_NOTNULL(crosswalks);
  const tinyxml2::XMLElement* sub_node = xml_node.FirstChildElement("object");
  while (sub_node) {
    std::string object_type;
    std::string object_id;
    int checker =
        UtilXmlParser::QueryStringAttribute(*sub_node, "type", &object_type);
    checker += UtilXmlParser::QueryStringAttribute(*sub_node, "id", &object_id);
    if (checker != tinyxml2::XML_SUCCESS) {
      std::string err_msg = "Error parse object type.";
      return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
    }

    if (object_type == "crosswalk") {
      PbCrosswalk crosswalk;
      crosswalk.mutable_id()->set_id(object_id);
      PbPolygon* polygon = crosswalk.mutable_polygon();
      const tinyxml2::XMLElement* outline_node =
          sub_node->FirstChildElement("outline");
      if (outline_node == nullptr) {
        std::string err_msg = "Error parse crosswalk outline";
        return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
      }
      RETURN_IF_ERROR(UtilXmlParser::ParseOutline(*outline_node, polygon));
      crosswalks->emplace_back(crosswalk);
    }
    sub_node = sub_node->NextSiblingElement("object");
  }
  return Status::OK();
}

Status ObjectsXmlParser::ParseClearAreas(
    const tinyxml2::XMLElement& xml_node,
    std::vector<PbClearArea>* clear_areas) {
  CHECK_NOTNULL(clear_areas);
  const tinyxml2::XMLElement* sub_node = xml_node.FirstChildElement("object");
  while (sub_node) {
    std::string object_type;
    std::string object_id;
    int checker =
        UtilXmlParser::QueryStringAttribute(*sub_node, "id", &object_id);
    checker +=
        UtilXmlParser::QueryStringAttribute(*sub_node, "type", &object_type);
    if (checker != tinyxml2::XML_SUCCESS) {
      std::string err_msg = "Error parse object type.";
      return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
    }

    if (object_type == "clearArea") {
      PbClearArea clear_area;
      clear_area.mutable_id()->set_id(object_id);
      PbPolygon* polygon = clear_area.mutable_polygon();
      ACHECK(polygon != nullptr);
      const tinyxml2::XMLElement* outline_node =
          sub_node->FirstChildElement("outline");
      if (outline_node == nullptr) {
        std::string err_msg = "Error parse cleararea outline";
        return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
      }
      RETURN_IF_ERROR(UtilXmlParser::ParseOutline(*outline_node, polygon));
      clear_areas->emplace_back(clear_area);
    }
    sub_node = sub_node->NextSiblingElement("object");
  }

  return Status::OK();
}

Status ObjectsXmlParser::ParseSpeedBumps(
    const tinyxml2::XMLElement& xml_node,
    std::vector<PbSpeedBump>* speed_bumps) {
  CHECK_NOTNULL(speed_bumps);
  const tinyxml2::XMLElement* object_node =
      xml_node.FirstChildElement("object");
  while (object_node) {
    std::string object_type;
    std::string object_id;
    int checker =
        UtilXmlParser::QueryStringAttribute(*object_node, "id", &object_id);
    checker +=
        UtilXmlParser::QueryStringAttribute(*object_node, "type", &object_type);
    if (checker != tinyxml2::XML_SUCCESS) {
      std::string err_msg = "Error parse object type.";
      return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
    }

    if (object_type == "speedBump") {
      PbSpeedBump speed_bump;
      const tinyxml2::XMLElement* sub_node =
          object_node->FirstChildElement("geometry");
      speed_bump.mutable_id()->set_id(object_id);
      while (sub_node) {
        PbCurve* curve = speed_bump.add_position();
        PbCurveSegment* curve_segment = curve->add_segment();
        RETURN_IF_ERROR(UtilXmlParser::ParseGeometry(*sub_node, curve_segment));
        sub_node = sub_node->NextSiblingElement("geometry");
      }
      if (speed_bump.position().empty()) {
        std::string err_msg = "Error speed bump miss stop line.";
        return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
      }
      speed_bumps->emplace_back(speed_bump);
    }
    object_node = object_node->NextSiblingElement("object");
  }
  return Status::OK();
}

Status ObjectsXmlParser::ParseStopLines(
    const tinyxml2::XMLElement& xml_node,
    std::vector<StopLineInternal>* stop_lines) {
  CHECK_NOTNULL(stop_lines);
  const tinyxml2::XMLElement* object_node =
      xml_node.FirstChildElement("object");
  while (object_node) {
    std::string object_type;
    std::string object_id;
    int checker =
        UtilXmlParser::QueryStringAttribute(*object_node, "id", &object_id);
    checker +=
        UtilXmlParser::QueryStringAttribute(*object_node, "type", &object_type);
    if (checker != tinyxml2::XML_SUCCESS) {
      std::string err_msg = "Error parse object type.";
      return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
    }

    if (object_type == "stopline") {
      StopLineInternal stop_line;
      stop_line.id = object_id;
      PbCurveSegment* curve_segment = stop_line.curve.add_segment();
      ACHECK(curve_segment != nullptr);
      const auto sub_node = object_node->FirstChildElement("geometry");
      if (sub_node == nullptr) {
        std::string err_msg = "Error parse stopline geometry";
        return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
      }
      RETURN_IF_ERROR(UtilXmlParser::ParseGeometry(*sub_node, curve_segment));
      stop_lines->emplace_back(stop_line);
    }
    object_node = object_node->NextSiblingElement("object");
  }
  return Status::OK();
}

Status ObjectsXmlParser::ParseParkingSpaces(
    const tinyxml2::XMLElement& xml_node,
    std::vector<PbParkingSpace>* parking_spaces) {
  CHECK_NOTNULL(parking_spaces);
  const tinyxml2::XMLElement* sub_node = xml_node.FirstChildElement("object");
  while (sub_node) {
    std::string object_type;
    std::string object_id;
    int checker =
        UtilXmlParser::QueryStringAttribute(*sub_node, "type", &object_type);
    checker += UtilXmlParser::QueryStringAttribute(*sub_node, "id", &object_id);
    if (checker != tinyxml2::XML_SUCCESS) {
      std::string err_msg = "Error parse object type.";
      return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
    }

    if (object_type == "parkingSpace") {
      PbParkingSpace parking_space;
      parking_space.mutable_id()->set_id(object_id);

      double heading = 0.0;
      checker = sub_node->QueryDoubleAttribute("heading", &heading);
      if (checker != tinyxml2::XML_SUCCESS) {
        std::string err_msg = "Error parse parking space heading.";
        return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
      }
      parking_space.set_heading(heading);

      PbPolygon* polygon = parking_space.mutable_polygon();
      const auto* outline_node = sub_node->FirstChildElement("outline");
      if (outline_node == nullptr) {
        std::string err_msg = "Error parse parking space outline";
        return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
      }
      RETURN_IF_ERROR(UtilXmlParser::ParseOutline(*outline_node, polygon));
      parking_spaces->emplace_back(parking_space);
    }
    sub_node = sub_node->NextSiblingElement("object");
  }
  return Status::OK();
}

Status ObjectsXmlParser::ParsePNCJunctions(
    const tinyxml2::XMLElement& xml_node,
    std::vector<PbPNCJunction>* pnc_junctions) {
  CHECK_NOTNULL(pnc_junctions);

  const tinyxml2::XMLElement* sub_node = xml_node.FirstChildElement("object");
  while (sub_node) {
    std::string object_type;
    std::string object_id;
    int checker =
        UtilXmlParser::QueryStringAttribute(*sub_node, "type", &object_type);
    checker += UtilXmlParser::QueryStringAttribute(*sub_node, "id", &object_id);
    if (checker != tinyxml2::XML_SUCCESS) {
      std::string err_msg = "Error parse object type.";
      return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
    }

    if (object_type == "PNCJunction") {
      PbPNCJunction pnc_junction;
      pnc_junction.mutable_id()->set_id(object_id);

      PbPolygon* polygon = pnc_junction.mutable_polygon();
      const auto* outline_node = sub_node->FirstChildElement("outline");
      if (outline_node == nullptr) {
        std::string err_msg = "Error parse pnc junction outline";
        return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
      }
      RETURN_IF_ERROR(UtilXmlParser::ParseOutline(*outline_node, polygon));

      RETURN_IF_ERROR(ParsePassageGroup(*sub_node, &pnc_junction));

      pnc_junctions->emplace_back(pnc_junction);
    }

    sub_node = sub_node->NextSiblingElement("object");
  }

  return Status::OK();
}

Status ObjectsXmlParser::ParsePassageGroup(const tinyxml2::XMLElement& xml_node,
                                           PbPNCJunction* pnc_junction) {
  CHECK_NOTNULL(pnc_junction);

  auto sub_node = xml_node.FirstChildElement("passageGroup");
  while (sub_node) {
    std::string object_id;
    std::string object_type;
    PbPassageGroup* passage_group = pnc_junction->add_passage_group();
    int checker =
        UtilXmlParser::QueryStringAttribute(*sub_node, "id", &object_id);
    if (checker != tinyxml2::XML_SUCCESS) {
      std::string err_msg = "Error parse object type.";
      return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
    }
    passage_group->mutable_id()->set_id(object_id);

    RETURN_IF_ERROR(ParsePassage(*sub_node, passage_group));

    sub_node = sub_node->NextSiblingElement("passageGroup");
  }

  return Status::OK();
}

Status ObjectsXmlParser::ParsePassage(const tinyxml2::XMLElement& xml_node,
                                      PbPassageGroup* passage_group) {
  CHECK_NOTNULL(passage_group);

  auto sub_node = xml_node.FirstChildElement("passage");
  while (sub_node) {
    std::string object_type;
    int checker =
        UtilXmlParser::QueryStringAttribute(*sub_node, "type", &object_type);
    if (checker != tinyxml2::XML_SUCCESS) {
      std::string err_msg = "Error parse object type.";
      return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
    }

    auto passage = passage_group->add_passage();
    PbPassageType pb_passage_type;
    RETURN_IF_ERROR(ToPassageType(object_type, &pb_passage_type));
    passage->set_type(pb_passage_type);

    std::vector<std::string> passage_node_ids;
    RETURN_IF_ERROR(ParsePassageIds(*sub_node, "laneID", &passage_node_ids));
    for (auto id : passage_node_ids) {
      passage->add_lane_id()->set_id(id);
    }

    RETURN_IF_ERROR(ParsePassageIds(*sub_node, "signalID", &passage_node_ids));
    for (auto id : passage_node_ids) {
      passage->add_signal_id()->set_id(id);
    }

    RETURN_IF_ERROR(ParsePassageIds(*sub_node, "yieldlID", &passage_node_ids));
    for (auto id : passage_node_ids) {
      passage->add_yield_id()->set_id(id);
    }

    RETURN_IF_ERROR(
        ParsePassageIds(*sub_node, "stopSignID", &passage_node_ids));
    for (auto id : passage_node_ids) {
      passage->add_stop_sign_id()->set_id(id);
    }

    sub_node = sub_node->NextSiblingElement("passage");
  }

  return Status::OK();
}

Status ObjectsXmlParser::ParsePassageIds(
    const tinyxml2::XMLElement& xml_node, const std::string& child_node_name,
    std::vector<std::string>* passage_node_ids) {
  CHECK_NOTNULL(passage_node_ids);

  passage_node_ids->clear();
  auto sub_node = xml_node.FirstChildElement(child_node_name.c_str());
  while (sub_node) {
    std::string object_id;
    int checker =
        UtilXmlParser::QueryStringAttribute(*sub_node, "id", &object_id);
    if (checker != tinyxml2::XML_SUCCESS) {
      std::string err_msg = "Error parse passage lane id.";
      return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
    }
    passage_node_ids->push_back(object_id);

    sub_node = sub_node->NextSiblingElement(child_node_name.c_str());
  }

  return Status::OK();
}

Status ObjectsXmlParser::ToPassageType(const std::string& type,
                                       PbPassageType* passage_type) {
  CHECK_NOTNULL(passage_type);

  std::string upper_str = UtilXmlParser::ToUpper(type);
  if (upper_str == "ENTRANCE") {
    *passage_type = apollo::hdmap::Passage_Type_ENTRANCE;
  } else if (upper_str == "EXIT") {
    *passage_type = apollo::hdmap::Passage_Type_EXIT;
  } else {
    *passage_type = apollo::hdmap::Passage_Type_UNKNOWN;
  }

  return Status::OK();
}

Status ObjectsXmlParser::ParseRSUs(
    const tinyxml2::XMLElement& xml_node,
    std::vector<RSUInternal>* rsus) {
  CHECK_NOTNULL(rsus);

  auto rsu_node = xml_node.FirstChildElement("object");
  while (rsu_node) {
    std::string object_type;
    std::string object_id;
    int checker =
        UtilXmlParser::QueryStringAttribute(*rsu_node, "type", &object_type);
    checker +=
        UtilXmlParser::QueryStringAttribute(*rsu_node, "id", &object_id);
    if (checker != tinyxml2::XML_SUCCESS) {
      std::string err_msg = "Error parse rsu type.";
      return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
    }

    if (object_type == "rsu") {
      std::string junction_id;
      checker = UtilXmlParser::QueryStringAttribute(*rsu_node,
                                            "junctionID", &junction_id);
      if (checker != tinyxml2::XML_SUCCESS) {
        std::string err_msg = "Error parse rsu junction id.";
        return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
      }

      RSUInternal rsu;
      rsu.rsu.mutable_id()->set_id(object_id);
      rsu.rsu.mutable_junction_id()->set_id(junction_id);
      rsu.id = object_id;
      rsus->emplace_back(rsu);

      rsu_node = rsu_node->NextSiblingElement("object");
    }
  }

  return Status::OK();
}

Status ObjectsXmlParser::ParseObjects(const tinyxml2::XMLElement& xml_node,
                                      ObjectInternal* objects) {
  CHECK_NOTNULL(objects);

  // objects
  auto sub_node = xml_node.FirstChildElement("objects");
  if (sub_node != nullptr) {
    // rsus
    RETURN_IF_ERROR(ObjectsXmlParser::ParseRSUs(*sub_node, &objects->rsus));
  }

  return Status::OK();
}

}  // namespace adapter
}  // namespace hdmap
}  // namespace apollo
