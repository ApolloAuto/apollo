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
#include "modules/bridge/common/util.h"

namespace apollo {
namespace bridge {

int GetProtoSize(const char *buf, size_t size) {
  if (size != sizeof(size_t)) {
    return 0;
  }
  char size_buf[sizeof(size_t)] = {0};
  memcpy(size_buf, buf, sizeof(size_t));
  int proto_size = *(reinterpret_cast<int *>(size_buf));
  return proto_size;
}

}  // namespace bridge
}  // namespace apollo
