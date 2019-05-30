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

#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/message.h>
#include <google/protobuf/text_format.h>
#include <condition_variable>
#include <fstream>
#include <memory>
#include <string>
#include <thread>
#include <type_traits>
#include <unordered_map>
#include <utility>

#include "cyber/common/log.h"
#include "cyber/record/file/record_file_base.h"
#include "cyber/record/file/section.h"
#include "cyber/time/time.h"

namespace apollo {
namespace cyber {
namespace record {

using google::protobuf::io::FileOutputStream;
using google::protobuf::io::ZeroCopyOutputStream;

struct Chunk {
  Chunk() { clear(); }

  inline void clear() {
    body_.reset(new ChunkBody());
    header_.set_begin_time(0);
    header_.set_end_time(0);
    header_.set_message_number(0);
    header_.set_raw_size(0);
  }

  inline void add(const SingleMessage& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    SingleMessage* p_message = body_->add_messages();
    *p_message = message;
    if (header_.begin_time() == 0) {
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
  std::unique_ptr<ChunkBody> body_ = nullptr;
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
  bool WriteChunk(const ChunkHeader& chunk_header, const ChunkBody& chunk_body);
  template <typename T>
  bool WriteSection(const T& message);
  bool WriteIndex();
  void Flush();
  bool is_writing_ = false;
  std::unique_ptr<Chunk> chunk_active_ = nullptr;
  std::unique_ptr<Chunk> chunk_flush_ = nullptr;
  std::shared_ptr<std::thread> flush_thread_ = nullptr;
  std::mutex flush_mutex_;
  std::condition_variable flush_cv_;
  std::unordered_map<std::string, uint64_t> channel_message_number_map_;
};

template <typename T>
bool RecordFileWriter::WriteSection(const T& message) {
  SectionType type;
  if (std::is_same<T, ChunkHeader>::value) {
    type = SectionType::SECTION_CHUNK_HEADER;
  } else if (std::is_same<T, ChunkBody>::value) {
    type = SectionType::SECTION_CHUNK_BODY;
  } else if (std::is_same<T, Channel>::value) {
    type = SectionType::SECTION_CHANNEL;
  } else if (std::is_same<T, Header>::value) {
    type = SectionType::SECTION_HEADER;
    if (!SetPosition(0)) {
      AERROR << "Jump to position #0 failed";
      return false;
    }
  } else if (std::is_same<T, Index>::value) {
    type = SectionType::SECTION_INDEX;
  } else {
    AERROR << "Do not support this template typename.";
    return false;
  }
  Section section = {type, message.ByteSize()};
  ssize_t count = write(fd_, &section, sizeof(section));
  if (count < 0) {
    AERROR << "Write fd failed, fd: " << fd_ << ", errno: " << errno;
    return false;
  }
  if (count != sizeof(section)) {
    AERROR << "Write fd failed, fd: " << fd_
           << ", expect count: " << sizeof(section)
           << ", actual count: " << count;
    return false;
  }
  ZeroCopyOutputStream* raw_output = new FileOutputStream(fd_);
  message.SerializeToZeroCopyStream(raw_output);
  delete raw_output;
  if (type == SectionType::SECTION_HEADER) {
    static char blank[HEADER_LENGTH] = {'0'};
    count = write(fd_, &blank, HEADER_LENGTH - message.ByteSize());
    if (count < 0) {
      AERROR << "Write fd failed, fd: " << fd_ << ", errno: " << errno;
      return false;
    }
    if (count != HEADER_LENGTH - message.ByteSize()) {
      AERROR << "Write fd failed, fd: " << fd_
             << ", expect count: " << sizeof(section)
             << ", actual count: " << count;
      return false;
    }
  }
  header_.set_size(CurrentPosition());
  return true;
}

}  // namespace record
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_RECORD_FILE_RECORD_FILE_WRITER_H_
