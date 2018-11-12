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

#ifndef CYBER_MESSAGE_INTRA_MESSAGE_H_
#define CYBER_MESSAGE_INTRA_MESSAGE_H_

#include <cassert>
#include <memory>
#include <string>

namespace apollo {
namespace cyber {
namespace message {

class IntraMessage {
 public:
  bool SerializeToArray(void *data, int size) const { return false; }

  bool SerializeToString(std::string *str) const { return false; }

  bool ParseFromArray(const void *data, int size) { return false; }

  bool ParseFromString(const std::string &str) { return false; }

  static std::string TypeName() { return std::string(type_name_); }

  int ByteSize() const { return -1; }

 protected:
  static const char *type_name_;
};

}  // namespace message
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_MESSAGE_INTRA_MESSAGE_H_
