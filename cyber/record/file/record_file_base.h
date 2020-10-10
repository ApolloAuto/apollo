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

#ifndef CYBER_RECORD_FILE_RECORD_FILE_BASE_H_
#define CYBER_RECORD_FILE_RECORD_FILE_BASE_H_

#include <mutex>
#include <string>

#include "cyber/proto/record.pb.h"

namespace apollo {
namespace cyber {
namespace record {

const int HEADER_LENGTH = 2048;

class RecordFileBase {
 public:
  RecordFileBase() = default;
  virtual ~RecordFileBase() = default;
  virtual bool Open(const std::string& path) = 0;
  virtual void Close() = 0;
  const std::string& GetPath() const { return path_; }
  const proto::Header& GetHeader() const { return header_; }
  const proto::Index& GetIndex() const { return index_; }
  int64_t CurrentPosition();
  bool SetPosition(int64_t position);

 protected:
  std::mutex mutex_;
  std::string path_;
  proto::Header header_;
  proto::Index index_;
  int fd_ = -1;
};

}  // namespace record
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_RECORD_FILE_RECORD_FILE_BASE_H_
