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

#include "cyber/binary.h"

#include <mutex>
#include <string>

namespace {
std::mutex m;
std::string binary_name; // NOLINT
}  // namespace

namespace apollo {
namespace cyber {
namespace binary {

std::string GetName() {
  std::lock_guard<std::mutex> lock(m);
  return binary_name;
}
void SetName(const std::string& name) {
  std::lock_guard<std::mutex> lock(m);
  binary_name = name;
}

}  // namespace binary
}  // namespace cyber
}  // namespace apollo
