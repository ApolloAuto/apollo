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

using ::apollo::cybertron::message::RawMessage;
using ::apollo::cybertron::record::RecordFileReader;

namespace apollo {
namespace cybertron {
namespace record {

class RecordReader : public RecordBase {
 public:
  RecordReader();
  virtual ~RecordReader();
  bool Open(const std::string& filename, uint64_t begin_time = 0,
            uint64_t end_time = UINT64_MAX);
  void Close();
  bool ReadMessage();
  bool EndOfFile();
  const std::string& CurrentMessageChannelName();
  std::shared_ptr<RawMessage> CurrentRawMessage();
  uint64_t CurrentMessageTime();

 private:
  bool InitLoadThread();
  void LoadChunk(uint64_t from_time, uint64_t to_time);

  bool is_reading_ = false;
  bool looped_readback_ = false;
  bool loaded_all_message_ = false;
  uint64_t begin_time_ = 0;
  uint64_t end_time_ = UINT64_MAX;
  uint32_t preload_seconds_ = 3;
  uint32_t min_queue_size_ = 2000;
  std::unique_ptr<RecordFileReader> file_reader_ = nullptr;
  std::shared_ptr<std::thread> load_thread_ = nullptr;

  struct RecordMessage {
   public:
    RecordMessage()
        : channel_name(""), raw_message(new RawMessage()), time(0) {}

    explicit RecordMessage(const std::string& channel_name,
                           std::shared_ptr<RawMessage>& raw_message,
                           uint64_t time)
        : channel_name(std::move(channel_name)),
          raw_message(raw_message),
          time(time) {}

    ~RecordMessage() {}

    std::string channel_name;
    std::shared_ptr<RawMessage> raw_message;
    uint64_t time;

   private:
    RecordMessage(const RecordMessage &) = delete;
    RecordMessage &operator=(const RecordMessage &) = delete;
  };
  std::shared_ptr<RecordMessage> current_message_;
  std::queue<std::shared_ptr<RecordMessage>> message_queue_;
};

}  // namespace record
}  // namespace cybertron
}  // namespace apollo

#endif  // CYBERTRON_RECORD_RECORD_READER_H_
