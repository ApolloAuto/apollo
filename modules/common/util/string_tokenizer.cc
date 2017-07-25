/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/common/util/string_tokenizer.h"

namespace apollo {
namespace common {
namespace util {

StringTokenizer::StringTokenizer(const std::string &s,
                                 const std::string &delims) {
  s_ = s;
  delims_ = delims;

  last_index_ = s_.find_first_not_of(delims, 0);
  index_ = s_.find_first_of(delims, last_index_);
}

std::vector<std::string> StringTokenizer::Split(const std::string &str,
                                                const std::string &delims) {
  std::vector<std::string> tokens;
  std::string::size_type last_index = str.find_first_not_of(delims, 0);
  std::string::size_type index = str.find_first_of(delims, last_index);

  while (std::string::npos != index || std::string::npos != last_index) {
    tokens.push_back(str.substr(last_index, index - last_index));
    last_index = str.find_first_not_of(delims, index);
    index = str.find_first_of(delims, last_index);
  }
  return tokens;
}

std::string StringTokenizer::Next() {
  if (std::string::npos != index_ || std::string::npos != last_index_) {
    auto token = s_.substr(last_index_, index_ - last_index_);
    last_index_ = s_.find_first_not_of(delims_, index_);
    index_ = s_.find_first_of(delims_, last_index_);
    return token;
  }
  return "";
}

}  // namespace util
}  // namespace common
}  // namespace apollo
