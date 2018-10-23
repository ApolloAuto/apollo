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

#ifndef CYBER_RECORD_FILE_RECORD_FILE_WRITER_H_
#define CYBER_RECORD_FILE_RECORD_FILE_WRITER_H_

#include <condition_variable>
#include <fstream>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include "cyber/common/log.h"
#include "cyber/record/file/record_file_base.h"
#include "cyber/record/file/section.h"

namespace apollo {
namespace cyber {
namespace record {

struct Chunk {
  Chunk() { clear(); }

  inline void clear() {
    body_.clear_messages();
    header_.set_begin_time(0);
    header_.set_end_time(0);
    header_.set_message_number(0);
    header_.set_raw_size(0);
  }

  inline void add(const SingleMessage& message) {
    std::lock_guard<std::mutex> lg(mutex_);
    SingleMessage* p_message = body_.add_messages();
    *p_message = message;
    if (0 == header_.begin_time()) {
      header_.set_begin_time(message.time());
    }
    if (header_.begin_time() > message.time()) {
      header_.set_begin_time(message.time());
    }
    if (header_.end_time() < message.time()) {
      header_.set_end_time(message.time());
    }
    header_.set_message_number(header_.message_number() + 1);
    header_.set_raw_size(header_.raw_size() + message.content().size());
  }

  inline bool empty() { return header_.message_number() == 0; }

  std::mutex mutex_;
  ChunkHeader header_;
  ChunkBody body_;
};

class RecordFileWriter : public RecordFileBase {
 public:
  RecordFileWriter();
  virtual ~RecordFileWriter();
  bool Open(const std::string& path) override;
  void Close() override;
  bool WriteHeader(const Header& header);
  bool WriteChannel(const Channel& channel);
  bool WriteMessage(const SingleMessage& message);
  uint64_t GetMessageNumber(const std::string& channel_name) const;

 private:
  void RefreshIndex();
  bool WriteIndex(const Index& index);
  bool WriteChunk(const ChunkHeader& chunk_header, const ChunkBody& chunk_body);
  void Flush();
  template <typename T>
  bool WriteSection(SectionType type, const T& message,
                    uint64_t fixed_size = 0);
  bool is_writing_ = false;
  std::unique_ptr<Chunk> chunk_active_ = nullptr;
  std::unique_ptr<Chunk> chunk_flush_ = nullptr;
  std::recursive_mutex chunk_mutex_;
  std::shared_ptr<std::thread> flush_thread_ = nullptr;
  std::mutex flush_mutex_;
  std::condition_variable flush_cv_;
  std::unordered_map<std::string, uint64_t> channel_message_number_map_;
  std::ofstream ofstream_;
};

template <typename T>
bool RecordFileWriter::WriteSection(SectionType type, const T& message,
                                    uint64_t fixed_size) {
  if (!ofstream_.is_open()) {
    AERROR << "ofstream if not open, file: " << path_;
    return false;
  }

  std::string message_str;
  if (!message.SerializeToString(&message_str)) {
    AERROR << "protobuf message serialize to string fail.";
    return false;
  }

  if (fixed_size > 0 && message_str.size() > fixed_size) {
    AERROR << "message string's size is already larger than fixed size"
           << ", size: " << message_str.size()
           << ", fixed size: " << fixed_size;
    return false;
  }

  Section section = {type, message_str.size()};
  ofstream_.write((const char*)&section, static_cast<int>(sizeof(section)));
  ofstream_.write((const char*)message_str.c_str(), message_str.size());

  if (fixed_size > 0) {
    static char blank[HEADER_LENGTH] = {'0'};
    ofstream_.write((const char*)blank, HEADER_LENGTH - message_str.size());
  }

  // ofstream_.flush();
  return true;
}

}  // namespace record
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_RECORD_FILE_RECORD_FILE_WRITER_H_
