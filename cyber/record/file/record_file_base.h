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

using ::apollo::cyber::proto::Channel;
using ::apollo::cyber::proto::ChannelCache;
using ::apollo::cyber::proto::ChunkBody;
using ::apollo::cyber::proto::ChunkBodyCache;
using ::apollo::cyber::proto::ChunkHeader;
using ::apollo::cyber::proto::ChunkHeaderCache;
using ::apollo::cyber::proto::CompressType;
using ::apollo::cyber::proto::Header;
using ::apollo::cyber::proto::Index;
using ::apollo::cyber::proto::SectionType;
using ::apollo::cyber::proto::SingleIndex;
using ::apollo::cyber::proto::SingleMessage;

class RecordFileBase {
 public:
  RecordFileBase() {}
  virtual ~RecordFileBase() {}
  virtual bool Open(const std::string& path) = 0;
  virtual void Close() = 0;
  std::string GetPath() { return path_; }
  Header GetHeader() { return header_; }
  Index GetIndex() { return index_; }
  int64_t CurrentPosition();
  bool SetPosition(int64_t position);

 protected:
  std::mutex mutex_;
  std::string path_;
  Header header_;
  Index index_;
  int fd_;
};

}  // namespace record
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_RECORD_FILE_RECORD_FILE_BASE_H_
