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

#include "cyber/transport/shm/readable_info.h"

#include <cstring>

#include "cyber/common/log.h"

namespace apollo {
namespace cyber {
namespace transport {

ReadableInfo::ReadableInfo() : host_id_(0), block_index_(0), channel_id_(0) {}

ReadableInfo::ReadableInfo(uint64_t host_id, uint32_t block_index,
                           uint64_t channel_id)
    : host_id_(host_id), block_index_(block_index), channel_id_(channel_id) {}

ReadableInfo::~ReadableInfo() {}

bool ReadableInfo::SerializeTo(std::string* dst) const {
  RETURN_VAL_IF_NULL(dst, false);
  dst->assign(reinterpret_cast<char*>(const_cast<uint64_t*>(&host_id_)),
              sizeof(host_id_));
  dst->append(reinterpret_cast<char*>(const_cast<uint32_t*>(&block_index_)),
              sizeof(block_index_));
  dst->append(reinterpret_cast<char*>(const_cast<uint64_t*>(&channel_id_)),
              sizeof(channel_id_));
  return true;
}

bool ReadableInfo::DeserializeFrom(const std::string& src) {
  if (src.size() != READABLE_INFO_SIZE) {
    AWARN << "src size[" << src.size() << "] mismatch.";
    return false;
  }

  char* ptr = const_cast<char*>(src.data());
  memcpy(reinterpret_cast<char*>(&host_id_), ptr, 8);
  ptr += 8;
  memcpy(reinterpret_cast<char*>(&block_index_), ptr, 4);
  ptr += 4;
  memcpy(reinterpret_cast<char*>(&channel_id_), ptr, 8);

  return true;
}

}  // namespace transport
}  // namespace cyber
}  // namespace apollo
