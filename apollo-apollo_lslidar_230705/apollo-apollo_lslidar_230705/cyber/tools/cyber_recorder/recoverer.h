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

#ifndef CYBER_TOOLS_CYBER_RECORDER_RECOVERER_H_
#define CYBER_TOOLS_CYBER_RECORDER_RECOVERER_H_

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "cyber/common/log.h"
#include "cyber/proto/record.pb.h"
#include "cyber/record/file/record_file_reader.h"
#include "cyber/record/file/record_file_writer.h"

using ::apollo::cyber::proto::ChannelCache;
using ::apollo::cyber::proto::ChunkBody;
using ::apollo::cyber::proto::ChunkHeader;
using ::apollo::cyber::proto::Header;

namespace apollo {
namespace cyber {
namespace record {

class Recoverer {
 public:
  Recoverer(const std::string& input_file, const std::string& output_file);
  virtual ~Recoverer();
  bool Proc();

 private:
  RecordFileReader reader_;
  RecordFileWriter writer_;
  std::string input_file_;
  std::string output_file_;
  std::vector<std::string> channel_vec_;
};

}  // namespace record
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_TOOLS_CYBER_RECORDER_RECOVERER_H_
