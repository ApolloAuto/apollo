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

#ifndef CYBER_TOOLS_CYBER_RECORDER_SPLITER_H_
#define CYBER_TOOLS_CYBER_RECORDER_SPLITER_H_

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "cyber/common/log.h"
#include "cyber/proto/record.pb.h"
#include "cyber/record/file/record_file_reader.h"
#include "cyber/record/file/record_file_writer.h"
#include "cyber/record/header_builder.h"

using ::apollo::cyber::proto::ChannelCache;
using ::apollo::cyber::proto::ChunkBody;
using ::apollo::cyber::proto::ChunkHeader;
using ::apollo::cyber::proto::Header;

namespace apollo {
namespace cyber {
namespace record {

class Spliter {
 public:
  Spliter(const std::string& input_file, const std::string& output_file,
          const std::vector<std::string>& white_channels,
          const std::vector<std::string>& black_channels,
          uint64_t begin_time = 0,
          uint64_t end_time = std::numeric_limits<uint64_t>::max());
  virtual ~Spliter();
  bool Proc();

 private:
  RecordFileReader reader_;
  RecordFileWriter writer_;
  std::string input_file_;
  std::string output_file_;
  std::vector<std::string> white_channels_;
  std::vector<std::string> black_channels_;
  bool all_channels_;
  uint64_t begin_time_;
  uint64_t end_time_;
};

}  // namespace record
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_TOOLS_CYBER_RECORDER_SPLITER_H_
