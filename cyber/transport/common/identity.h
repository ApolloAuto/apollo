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

#ifndef CYBER_TRANSPORT_COMMON_IDENTITY_H_
#define CYBER_TRANSPORT_COMMON_IDENTITY_H_

#include <cstdint>
#include <cstring>
#include <string>

namespace apollo {
namespace cyber {
namespace transport {

const uint8_t ID_SIZE = 8;

class Identity {
 public:
  explicit Identity(bool need_generate = true);
  Identity(const Identity& another);
  virtual ~Identity();

  Identity& operator=(const Identity& another);
  bool operator==(const Identity& another) const;
  bool operator!=(const Identity& another) const;

  const std::string& ToString() const;
  size_t Length() const;
  uint64_t HashValue() const;

  // getter and setter
  const char* data() const { return data_; }
  void set_data(const char* data) {
    if (data == nullptr) {
      return;
    }
    memset(data_, 0, sizeof(data_));
    memcpy(data_, data, sizeof(data_));
    Update();
  }

 private:
  void Update();

  char data_[ID_SIZE];
  uint64_t hash_value_;
  std::string hash_value_str_;
};

}  // namespace transport
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_TRANSPORT_COMMON_IDENTITY_H_
