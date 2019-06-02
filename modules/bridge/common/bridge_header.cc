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

#include "modules/bridge/common/bridge_header.h"

#include <cstring>

namespace apollo {
namespace bridge {

bool BridgeHeader::Serialize(char *buf, size_t size) {
  if (!buf || size == 0) {
    return false;
  }
  char *cursor = buf;
  char *p_header_size = nullptr;
  cursor = SerializeHeaderFlag(cursor, size);
  p_header_size = cursor;
  cursor += sizeof(size_t) + 1;
  for (int i = 0; i < Header_Tail; i++) {
    cursor = header_item[i]->SerializeItem(cursor, size, &header_size_);
  }

  if (!SerializeHeaderSize(p_header_size, size)) {
    return false;
  }
  return true;
}

bool BridgeHeader::Diserialize(const char *buf) {
  if (!IsAvailable(buf)) {
    return false;
  }
  const char *cursor = buf + sizeof(BRIDGE_HEADER_FLAG) + 1;

  if (!DiserializeBasicType<size_t, sizeof(size_t)>(&header_size_, cursor)) {
    return false;
  }
  cursor += sizeof(size_t) + 1;

  size_t i = header_size_ - sizeof(BRIDGE_HEADER_FLAG) - sizeof(size_t) - 2;
  while (i >= 0) {
    HType type = *(reinterpret_cast<const HType *>(cursor));
    if (type > Header_Tail || type < 0) {
      cursor += sizeof(HType) + 1;
      size_t size = *(reinterpret_cast<const size_t *>(cursor));
      cursor += sizeof(size_t) + size + 2;
      i -= sizeof(HType) + sizeof(size_t) + size + 3;
      continue;
    } else {
      size_t value_size = 0;
      for (int i = 0; i < Header_Tail; i++) {
        if (type == header_item[i]->GetType()) {
          cursor = header_item[i]->DiserializeItem(cursor, &value_size);
        }
      }
      i -= value_size;
    }
  }
  return true;
}

bool BridgeHeader::IsAvailable(const char *buf) {
  if (!buf) {
    return false;
  }
  if (memcmp(BRIDGE_HEADER_FLAG, buf, sizeof(BRIDGE_HEADER_FLAG) - 1) != 0) {
    return false;
  }
  return true;
}

char *BridgeHeader::SerializeHeaderFlag(char *buf, size_t size) {
  if (!buf || size == 0) {
    return nullptr;
  }
  return SerializeBasicType<char, sizeof(BRIDGE_HEADER_FLAG)>(
    BRIDGE_HEADER_FLAG, buf, size);
}


char *BridgeHeader::SerializeHeaderSize(char *buf, size_t size) {
  return SerializeBasicType<size_t, sizeof(size_t)>(&header_size_, buf, size);
}

}  // namespace bridge
}  // namespace apollo
