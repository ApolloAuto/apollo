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

#ifndef CYBERTRON_MESSAGE_INTRA_MESSAGE_H_
#define CYBERTRON_MESSAGE_INTRA_MESSAGE_H_

#include <cassert>
#include <memory>
#include <string>

namespace apollo {
namespace cybertron {
namespace message {

class IntraMessage {
 public:
  bool SerializeToString(std::string *str) const { return false; }

  bool ParseFromString(const std::string &str) { return false; }

  static std::string GetDescriptorString() { return ""; }

  static const std::string& TypeName() {
    return type_name_;
  }

 protected:
  static std::string type_name_;
};

}  // namespace message
}  // namespace cybertron
}  // namespace apollo

#endif  // CYBERTRON_MESSAGE_INTRA_MESSAGE_H_
