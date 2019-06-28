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
#include "modules/map/tools/map_datachecker/client/client_common.h"

#include <grpc++/grpc++.h>
#include <boost/algorithm/string.hpp>
#include <fstream>
#include <string>
#include <vector>

namespace apollo {
namespace hdmap {
std::vector<std::string> GetFileLines(const std::string& path) {
  std::ifstream file_handler(path);
  std::string line;
  std::vector<std::string> lines;
  while (std::getline(file_handler, line)) {
    lines.emplace_back(line);
  }
  file_handler.close();
  return lines;
}

}  // namespace hdmap
}  // namespace apollo
