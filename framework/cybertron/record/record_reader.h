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

#ifndef CYBERTRON_RECORD_RECORD_READER_H_
#define CYBERTRON_RECORD_RECORD_READER_H_

#include <algorithm>
#include <condition_variable>
#include <memory>
#include <queue>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>
#include "cybertron/message/raw_message.h"
#include "cybertron/record/record_base.h"
#include "cybertron/record/record_message.h"

namespace apollo {
namespace cybertron {
namespace record {

using ::apollo::cybertron::message::RawMessage;
using ::apollo::cybertron::record::RecordFileReader;

class RecordReader : public RecordBase {
 public:
  explicit RecordReader(const std::string& file);
  virtual ~RecordReader();
  bool ReadMessage(RecordMessage* message, uint64_t begin_time = 0,
                   uint64_t end_time = UINT64_MAX);
  std::set<std::string> GetChannelList() const;
  const Header& header() const;
  void Reset();

 private:
  bool ReadNextChunk(ChunkBody* chunk, uint64_t begin_time, uint64_t end_time);
  proto::ChunkBody chunk_;
  proto::Index index_;
  uint32_t message_index_ = 0;
  std::unique_ptr<RecordFileReader> file_reader_;
};

}  // namespace record
}  // namespace cybertron
}  // namespace apollo

#endif  // CYBERTRON_RECORD_RECORD_READER_H_
