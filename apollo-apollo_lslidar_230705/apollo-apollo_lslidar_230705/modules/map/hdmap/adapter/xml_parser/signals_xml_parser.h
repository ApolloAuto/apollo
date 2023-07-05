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
#pragma once

#include <string>
#include <vector>

#include <tinyxml2.h>

#include "modules/map/hdmap/adapter/xml_parser/common_define.h"
#include "modules/map/hdmap/adapter/xml_parser/status.h"

namespace apollo {
namespace hdmap {
namespace adapter {

class SignalsXmlParser {
 public:
  static Status ParseTrafficLights(
      const tinyxml2::XMLElement& xml_node,
      std::vector<TrafficLightInternal>* traffic_lights);
  static Status ParseStopSigns(const tinyxml2::XMLElement& xml_node,
                               std::vector<StopSignInternal>* stop_signs);
  static Status ParseYieldSigns(const tinyxml2::XMLElement& xml_node,
                                std::vector<YieldSignInternal>* yield_signs);

 private:
  static Status ToPbSignalType(const std::string& xml_type,
                               PbSignalType* signal_type);
  static Status ToPbSubSignalType(const std::string& xml_type,
                                  PbSubSignalType* sub_signal_type);
  static Status to_pb_sign_info_type(const std::string& xml_type,
                                     PbSignInfoType* sign_info_type);
  static Status ToPbStopSignType(const std::string& xml_type,
                                 PbStopSignType* stop_type);
};

}  // namespace adapter
}  // namespace hdmap
}  // namespace apollo
