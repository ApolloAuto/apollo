/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/common/util/yaml_util.h"

#include <google/protobuf/util/json_util.h>
#include <yaml-cpp/yaml.h>

namespace apollo {
namespace common {
namespace util {
namespace {

void YamlNodeToJson(const YAML::Node& node, std::ostringstream* oss) {
  switch (node.Type()) {
    case YAML::NodeType::Undefined:
      *oss << "\"\"";
      break;
    case YAML::NodeType::Null:
      *oss << "\"\"";
      break;
    case YAML::NodeType::Scalar:
      *oss << "\"" << node.as<std::string>() << "\"";
      break;
    case YAML::NodeType::Sequence:
      *oss << "[\n";
      for (const auto& sub_node : node) {
        YamlNodeToJson(sub_node, oss);
        *oss << ", ";
      }
      *oss << "]\n";
      break;
    case YAML::NodeType::Map:
      *oss << "{\n";
      for (const auto& sub_node : node) {
        YamlNodeToJson(sub_node.first, oss);
        *oss << ": ";
        YamlNodeToJson(sub_node.second, oss);
        *oss << ",\n";
      }
      *oss << "}\n";
      break;
  }
}

}  // namespace

// Load YAML file to an equivalent json string.
std::string YamlToJson(const std::string& yaml_file) {
  std::ostringstream oss;
  YamlNodeToJson(YAML::LoadFile(yaml_file), &oss);
  return oss.str();
}

// Load YAML file to an equivalent proto message.
// Return false if failed.
bool YamlToProto(const std::string& yaml_file,
                 google::protobuf::Message* proto) {
  const std::string json = YamlToJson(yaml_file);
  return google::protobuf::util::JsonStringToMessage(json, proto).ok();
}

}  // namespace util
}  // namespace common
}  // namespace apollo
