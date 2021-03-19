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
#pragma once
#include <iostream>
#include <map>
#include <set>
#include <string>

namespace apollo {
namespace perception {
namespace benchmark {

class OptionParser {
 public:
  OptionParser() = default;
  virtual ~OptionParser() = default;
  bool parse_from_string(const std::string& input);
  virtual bool set_options() const { return true; }
  friend std::ostream& operator<<(std::ostream& out, const OptionParser& rhs);

 protected:
  std::map<std::string, std::set<std::string>> _options;
};

}  // namespace benchmark
}  // namespace perception
}  // namespace apollo
