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

#include "cyber/transport/common/identity.h"

#include <uuid/uuid.h>

#include "cyber/common/util.h"

namespace apollo {
namespace cyber {
namespace transport {

Identity::Identity(bool need_generate) : hash_value_(0), hash_value_str_("") {
  memset(data_, 0, ID_SIZE);
  if (need_generate) {
    uuid_t uuid;
    uuid_generate(uuid);
    memcpy(data_, uuid, ID_SIZE);
    Update();
  }
}

Identity::Identity(const Identity& another) {
  memcpy(data_, another.data(), ID_SIZE);
  hash_value_ = another.hash_value_;
  hash_value_str_ = another.hash_value_str_;
}

Identity::~Identity() {}

Identity& Identity::operator=(const Identity& another) {
  if (this != &another) {
    memcpy(data_, another.data(), ID_SIZE);
    hash_value_ = another.hash_value_;
    hash_value_str_ = another.hash_value_str_;
  }
  return *this;
}

bool Identity::operator==(const Identity& another) const {
  return memcmp(data_, another.data(), ID_SIZE) == 0;
}

bool Identity::operator!=(const Identity& another) const {
  return memcmp(data_, another.data(), ID_SIZE) != 0;
}

const std::string& Identity::ToString() const { return hash_value_str_; }

size_t Identity::Length() const { return ID_SIZE; }

uint64_t Identity::HashValue() const { return hash_value_; }

void Identity::Update() {
  hash_value_ = common::Hash(std::string(data_, ID_SIZE));
  hash_value_str_ = std::to_string(hash_value_);
}

}  // namespace transport
}  // namespace cyber
}  // namespace apollo
