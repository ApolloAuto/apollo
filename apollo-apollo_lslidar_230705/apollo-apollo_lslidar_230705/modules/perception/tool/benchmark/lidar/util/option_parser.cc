/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/tool/benchmark/lidar/util/option_parser.h"

#include <vector>

#include "absl/strings/str_split.h"
#include "modules/common/util/string_util.h"

namespace apollo {
namespace perception {
namespace benchmark {

bool OptionParser::parse_from_string(const std::string& input) {
  _options.clear();
  std::vector<std::string> option_pairs = absl::StrSplit(input, '|');
  // std::string str = StringUtil::trim_all(input);
  if (option_pairs.empty()) {  // single option
    option_pairs.push_back(input);
  }
  for (auto& opt : option_pairs) {
    const std::vector<std::string> key_value = absl::StrSplit(opt, ':');
    if (key_value.size() != 2) {
      std::cerr << "Fail to parse " << opt << std::endl;
      continue;
    }
    std::vector<std::string> values = absl::StrSplit(key_value[1], ',');
    if (values.empty()) {
      values.push_back(key_value[1]);
    }
    auto iter = _options.find(key_value[0]);
    if (iter != _options.end()) {
      for (auto& value : values) {
        iter->second.insert(value);
      }
    } else {
      std::set<std::string> value_set;
      for (auto& value : values) {
        value_set.insert(value);
      }
      _options.emplace(key_value[0], value_set);
    }
  }
  return true;
}

std::ostream& operator<<(std::ostream& out, const OptionParser& rhs) {
  out << "Options: " << std::endl;
  for (auto& pair : rhs._options) {
    out << pair.first << ":   ";
    for (auto& value : pair.second) {
      out << value << " ";
    }
    out << std::endl;
  }
  return out;
}

}  // namespace benchmark
}  // namespace perception
}  // namespace apollo
