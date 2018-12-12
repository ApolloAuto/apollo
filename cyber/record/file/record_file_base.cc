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

#include "cyber/record/file/record_file_base.h"

#include <sys/types.h>
#include <unistd.h>

#include "cyber/common/log.h"

namespace apollo {
namespace cyber {
namespace record {

uint64_t RecordFileBase::CurrentPosition() {
  off_t pos = lseek(fd_, 0, SEEK_CUR);
  if (pos < 0) {
    AERROR << "lseek failed, file: " << path_ << ", fd: " << fd_
           << ", offset: 0, whence: SEEK_CUR"
           << ", position: " << pos << ", errno: " << errno;
  }
  return pos;
}

bool RecordFileBase::SetPosition(uint64_t position) {
  if (position > INT64_MAX) {
    AERROR << "position > INT64_MAX";
    return false;
  }
  off_t pos = lseek(fd_, position, SEEK_SET);
  if (pos < 0) {
    AERROR << "lseek failed, file: " << path_ << ", fd: " << fd_
           << ", offset: 0, whence: SEEK_SET"
           << ", position: " << pos << ", errno: " << errno;
    return false;
  }
  return true;
}

}  // namespace record
}  // namespace cyber
}  // namespace apollo
