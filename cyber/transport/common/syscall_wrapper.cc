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

#include "cyber/transport/common/syscall_wrapper.h"

#include "cyber/common/log.h"

namespace apollo {
namespace cyber {
namespace transport {

bool SetNonBlocking(int fd) {
  if (fcntl(fd, F_SETFL, O_NONBLOCK) == -1) {
    AERROR << "set non block failed.";
    return false;
  }
  return true;
}

void CloseAndReset(int *fd) {
  RETURN_IF_NULL(fd);
  if (*fd < 0) {
    return;
  }
  close(*fd);
  *fd = -1;
}

}  // namespace transport
}  // namespace cyber
}  // namespace apollo
