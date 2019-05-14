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
#include "modules/map/hdmap/adapter/xml_parser/signals_xml_parser.h"

#include "modules/map/hdmap/adapter/xml_parser/util_xml_parser.h"

namespace apollo {
namespace hdmap {
namespace adapter {

Status SignalsXmlParser::ParseTrafficLights(
    const tinyxml2::XMLElement& xml_node,
    std::vector<TrafficLightInternal>* traffic_lights) {
  CHECK_NOTNULL(traffic_lights);
  auto signal_node = xml_node.FirstChildElement("signal");
  while (signal_node) {
    std::string object_type;
    std::string object_id;
    int checker =
        UtilXmlParser::QueryStringAttribute(*signal_node, "type", &object_type);
    checker +=
        UtilXmlParser::QueryStringAttribute(*signal_node, "id", &object_id);
    if (checker != tinyxml2::XML_SUCCESS) {
      std::string err_msg = "Error parse signal type.";
      return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
    }

    if (object_type == "trafficLight") {
      PbSignal traffic_light;
      traffic_light.mutable_id()->set_id(object_id);
      std::string layout_type;
      int checker = UtilXmlParser::QueryStringAttribute(
          *signal_node, "layoutType", &layout_type);
      if (checker != tinyxml2::XML_SUCCESS) {
        std::string err_msg = "Error parse signal layout type.";
        return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
      }
      PbSignalType signal_layout_type;
      ToPbSignalType(layout_type, &signal_layout_type);
      traffic_light.set_type(signal_layout_type);

      auto sign_infos_node = signal_node->FirstChildElement("signInfos");
      if (sign_infos_node != nullptr) {
        auto sign_info_node = sign_infos_node->FirstChildElement("signInfo");
        while (sign_info_node != nullptr) {
          std::string sign_info_type;
          checker = UtilXmlParser::QueryStringAttribute(*sign_info_node, "type",
                                                        &sign_info_type);
          if (checker != tinyxml2::XML_SUCCESS) {
            std::string err_msg = "Error parse sign info type.";
            return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
          }

          auto sign_info = traffic_light.add_sign_info();
          PbSignInfoType pb_sign_info_type;
          to_pb_sign_info_type(sign_info_type, &pb_sign_info_type);
          sign_info->set_type(pb_sign_info_type);

          sign_info_node = sign_info_node->NextSiblingElement("signInfo");
        }
      }

      PbPolygon* polygon = traffic_light.mutable_boundary();
      auto outline_node = signal_node->FirstChildElement("outline");
      if (outline_node == nullptr) {
        std::string err_msg = "Error parse signal outline";
        return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
      }
      RETURN_IF_ERROR(UtilXmlParser::ParseOutline(*outline_node, polygon));
      auto sub_node = signal_node->FirstChildElement("subSignal");
      if (sub_node == nullptr) {
        std::string err_msg = "Error parse sub signal.";
        return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
      }

      while (sub_node) {
        std::string sub_signal_id;
        std::string sub_signal_xml_type;
        checker = UtilXmlParser::QueryStringAttribute(*sub_node, "type",
                                                      &sub_signal_xml_type);
        checker += UtilXmlParser::QueryStringAttribute(*sub_node, "id",
                                                       &sub_signal_id);
        if (checker != tinyxml2::XML_SUCCESS) {
          std::string err_msg = "Error parse sub signal layout type.";
          return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
        }

        PbSubSignal* sub_signal = traffic_light.add_subsignal();
        PbSubSignalType sub_signal_type;
        ToPbSubSignalType(sub_signal_xml_type, &sub_signal_type);
        sub_signal->mutable_id()->set_id(sub_signal_id);
        sub_signal->set_type(sub_signal_type);
        PbPoint3D* pt = sub_signal->mutable_location();
        RETURN_IF_ERROR(UtilXmlParser::ParsePoint(*sub_node, pt));

        sub_node = sub_node->NextSiblingElement("subSignal");
      }

      TrafficLightInternal trafficlight_internal;
      trafficlight_internal.id = object_id;
      trafficlight_internal.traffic_light = traffic_light;

      sub_node = signal_node->FirstChildElement("stopline");
      if (sub_node) {
        sub_node = sub_node->FirstChildElement("objectReference");
        while (sub_node) {
          std::string stop_line_id;
          int checker = UtilXmlParser::QueryStringAttribute(*sub_node, "id",
                                                            &stop_line_id);
          CHECK(checker == tinyxml2::XML_SUCCESS);
          trafficlight_internal.stop_line_ids.insert(stop_line_id);
          sub_node = sub_node->NextSiblingElement("objectReference");
        }
      }
      traffic_lights->emplace_back(trafficlight_internal);
    }
    signal_node = signal_node->NextSiblingElement("signal");
  }
  return Status::OK();
}

Status SignalsXmlParser::ToPbSignalType(const std::string& xml_type,
                                        PbSignalType* signal_type) {
  CHECK_NOTNULL(signal_type);

  std::string upper_str = UtilXmlParser::ToUpper(xml_type);

  if (upper_str == "UNKNOWN") {
    *signal_type = hdmap::Signal::UNKNOWN;
  } else if (upper_str == "MIX2HORIZONTAL") {
    *signal_type = hdmap::Signal::MIX_2_HORIZONTAL;
  } else if (upper_str == "MIX2VERTICAL") {
    *signal_type = hdmap::Signal::MIX_2_VERTICAL;
  } else if (upper_str == "MIX3HORIZONTAL") {
    *signal_type = hdmap::Signal::MIX_3_HORIZONTAL;
  } else if (upper_str == "MIX3VERTICAL") {
    *signal_type = hdmap::Signal::MIX_3_VERTICAL;
  } else if (upper_str == "SINGLE") {
    *signal_type = hdmap::Signal::SINGLE;
  } else {
    std::string err_msg = "Error or unsupport signal layout type";
    return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
  }

  return Status::OK();
}

Status SignalsXmlParser::ToPbSubSignalType(const std::string& xml_type,
                                           PbSubSignalType* sub_signal_type) {
  CHECK_NOTNULL(sub_signal_type);

  std::string upper_str = UtilXmlParser::ToUpper(xml_type);

  if (upper_str == "UNKNOWN") {
    *sub_signal_type = hdmap::Subsignal::UNKNOWN;
  } else if (upper_str == "CIRCLE") {
    *sub_signal_type = hdmap::Subsignal::CIRCLE;
  } else if (upper_str == "ARROWLEFT") {
    *sub_signal_type = hdmap::Subsignal::ARROW_LEFT;
  } else if (upper_str == "ARROWFORWARD") {
    *sub_signal_type = hdmap::Subsignal::ARROW_FORWARD;
  } else if (upper_str == "ARROWRIGHT") {
    *sub_signal_type = hdmap::Subsignal::ARROW_RIGHT;
  } else if (upper_str == "ARROWLEFTANDFORWARD") {
    *sub_signal_type = hdmap::Subsignal::ARROW_LEFT_AND_FORWARD;
  } else if (upper_str == "ARROWRIGHTANDFORWARD") {
    *sub_signal_type = hdmap::Subsignal::ARROW_RIGHT_AND_FORWARD;
  } else if (upper_str == "ARROWUTURN") {
    *sub_signal_type = hdmap::Subsignal::ARROW_U_TURN;
  } else {
    std::string err_msg = "Error or unsupport sub signal type";
    return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
  }
  return Status::OK();
}

Status SignalsXmlParser::to_pb_sign_info_type(const std::string& xml_type,
                                              PbSignInfoType* sign_info_type) {
  CHECK_NOTNULL(sign_info_type);

  std::string upper_str = UtilXmlParser::ToUpper(xml_type);

  if (upper_str == "NORIGHTTURNONRED") {
    *sign_info_type = hdmap::SignInfo::NO_RIGHT_TURN_ON_RED;
  } else {
    std::string err_msg = "Error or unsupport stop sign type";
    return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
  }
  return Status::OK();
}

Status SignalsXmlParser::ToPbStopSignType(const std::string& xml_type,
                                          PbStopSignType* stop_type) {
  CHECK_NOTNULL(stop_type);

  std::string upper_str = UtilXmlParser::ToUpper(xml_type);

  if (upper_str == "UNKNOWN") {
    *stop_type = hdmap::StopSign::UNKNOWN;
  } else if (upper_str == "ONEWAY") {
    *stop_type = hdmap::StopSign::ONE_WAY;
  } else if (upper_str == "TWOWAY") {
    *stop_type = hdmap::StopSign::TWO_WAY;
  } else if (upper_str == "THREEWAY") {
    *stop_type = hdmap::StopSign::THREE_WAY;
  } else if (upper_str == "FOURWAY") {
    *stop_type = hdmap::StopSign::FOUR_WAY;
  } else if (upper_str == "ALLWAY") {
    *stop_type = hdmap::StopSign::ALL_WAY;
  } else {
    std::string err_msg = "Error or unsupport stop sign type";
    return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
  }
  return Status::OK();
}

Status SignalsXmlParser::ParseStopSigns(
    const tinyxml2::XMLElement& xml_node,
    std::vector<StopSignInternal>* stop_signs) {
  CHECK_NOTNULL(stop_signs);

  auto signal_node = xml_node.FirstChildElement("signal");
  while (signal_node) {
    std::string object_type;
    std::string object_id;
    int checker =
        UtilXmlParser::QueryStringAttribute(*signal_node, "type", &object_type);
    checker +=
        UtilXmlParser::QueryStringAttribute(*signal_node, "id", &object_id);
    if (checker != tinyxml2::XML_SUCCESS) {
      std::string err_msg = "Error parse signal type.";
      return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
    }

    if (object_type == "stopSign") {
      PbStopSign stop_sign;
      stop_sign.mutable_id()->set_id(object_id);

      StopSignInternal stop_sign_internal;
      stop_sign_internal.stop_sign = stop_sign;

      auto sub_node = signal_node->FirstChildElement("stopline");
      if (sub_node) {
        sub_node = sub_node->FirstChildElement("objectReference");
        while (sub_node) {
          std::string stop_line_id;
          int checker = UtilXmlParser::QueryStringAttribute(*sub_node, "id",
                                                            &stop_line_id);
          CHECK(checker == tinyxml2::XML_SUCCESS);
          stop_sign_internal.stop_line_ids.insert(stop_line_id);

          sub_node = sub_node->NextSiblingElement("objectReference");
        }
      }

      sub_node = signal_node->FirstChildElement("attribute");
      if (sub_node) {
        std::string stop_type;
        int checker = UtilXmlParser::QueryStringAttribute(*sub_node, "stopType",
                                                          &stop_type);
        if (checker != tinyxml2::XML_SUCCESS) {
          std::string err_msg = "Error parse stop type.";
          return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
        }

        PbStopSignType pb_stop_type;
        ToPbStopSignType(stop_type, &pb_stop_type);
        stop_sign_internal.stop_sign.set_type(pb_stop_type);
      }

      stop_signs->emplace_back(stop_sign_internal);
    }

    signal_node = signal_node->NextSiblingElement("signal");
  }
  return Status::OK();
}

Status SignalsXmlParser::ParseYieldSigns(
    const tinyxml2::XMLElement& xml_node,
    std::vector<YieldSignInternal>* yield_signs) {
  CHECK_NOTNULL(yield_signs);

  auto signal_node = xml_node.FirstChildElement("signal");
  while (signal_node) {
    std::string object_type;
    std::string object_id;
    int checker =
        UtilXmlParser::QueryStringAttribute(*signal_node, "type", &object_type);
    checker +=
        UtilXmlParser::QueryStringAttribute(*signal_node, "id", &object_id);
    if (checker != tinyxml2::XML_SUCCESS) {
      std::string err_msg = "Error parse signal type.";
      return Status(apollo::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
    }

    if (object_type == "yieldSign") {
      PbYieldSign yield_sign;
      yield_sign.mutable_id()->set_id(object_id);
      YieldSignInternal yield_sign_internal;
      yield_sign_internal.id = object_id;
      yield_sign_internal.yield_sign = yield_sign;
      auto sub_node = signal_node->FirstChildElement("stopline");
      if (sub_node) {
        sub_node = sub_node->FirstChildElement("objectReference");
        while (sub_node) {
          std::string stop_line_id;
          int checker = UtilXmlParser::QueryStringAttribute(*sub_node, "id",
                                                            &stop_line_id);
          CHECK(checker == tinyxml2::XML_SUCCESS);
          yield_sign_internal.stop_line_ids.insert(stop_line_id);

          sub_node = sub_node->NextSiblingElement("objectReference");
        }
      }
      yield_signs->emplace_back(yield_sign_internal);
    }

    signal_node = signal_node->NextSiblingElement("signal");
  }
  return Status::OK();
}

}  // namespace adapter
}  // namespace hdmap
}  // namespace apollo
