/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#ifndef CYBERTRON_RECORD_RECORD_BASE_H_
#define CYBERTRON_RECORD_RECORD_BASE_H_

#include <condition_variable>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>

#include "cybertron/common/file.h"
#include "cybertron/common/log.h"
#include "cybertron/proto/record.pb.h"

namespace apollo {
namespace cybertron {
namespace record {

const int HEADER_LENGTH = 2048;

using ::apollo::cybertron::proto::Header;
using ::apollo::cybertron::proto::Channel;
using ::apollo::cybertron::proto::ChunkHeader;
using ::apollo::cybertron::proto::ChunkBody;
using ::apollo::cybertron::proto::SingleMessage;
using ::apollo::cybertron::proto::Index;
using ::apollo::cybertron::proto::SingleIndex;
using ::apollo::cybertron::proto::ChannelCache;
using ::apollo::cybertron::proto::ChunkHeaderCache;
using ::apollo::cybertron::proto::ChunkBodyCache;
using ::apollo::cybertron::proto::SectionType;
using ::apollo::cybertron::proto::CompressType;

struct Section {
  SectionType type;
  uint64_t size;
};

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
    header_.set_end_time(message.time());
    header_.set_message_number(header_.message_number() + 1);
    header_.set_raw_size(header_.raw_size() + message.content().size());
  }

  inline bool empty() { return header_.message_number() == 0; }

  std::mutex mutex_;
  ChunkHeader header_;
  ChunkBody body_;
};

class RecordFile {
 public:
  RecordFile();
  virtual ~RecordFile();
  virtual bool Open(const std::string& path) = 0;
  virtual void Close() = 0;
  Header GetHeader();
  Index GetIndex();

 protected:
  std::mutex mutex_;
  std::string path_;
  Header header_;
  Index index_;
};

class RecordFileReader : public RecordFile {
 public:
  RecordFileReader();
  virtual ~RecordFileReader();
  bool Open(const std::string& path) override;
  void Close() override;

  bool ReadSection(Section* section);
  template <typename T>
  bool ReadSection(uint64_t size, T* message, uint64_t fixed_size = 0);
  bool ReadHeader();
  bool ReadIndex();
  bool EndOfFile();

 private:
  std::ifstream ifstream_;
};

template <typename T>
bool RecordFileReader::ReadSection(uint64_t size, T* message,
                                   uint64_t fixed_size) {
  if (size == 0) {
    return true;
  }
  std::string str;
  str.resize(size);
  if (0 < fixed_size && fixed_size < size) {
    AERROR << "size is larger than fixed size, size: " << size
           << ", fixed size: " << fixed_size;
    return false;
  }
  int64_t backup_position = ifstream_.tellg();
  ifstream_.read(reinterpret_cast<char*>(const_cast<char*>(str.c_str())), size);
  if (ifstream_.gcount() != size) {
    AERROR << "read section message fail, expect size: " << size
           << ", actual size: " << ifstream_.gcount();
    return false;
  }
  if (fixed_size > size) {
    ifstream_.seekg(backup_position + fixed_size, std::ios::beg);
  }
  if (!message->ParseFromString(str)) {
    AERROR << "parse message from string fail";
    return false;
  }
  return true;
}

class RecordFileWriter : public RecordFile {
 public:
  RecordFileWriter();
  virtual ~RecordFileWriter();
  bool Open(const std::string& path) override;
  void Close() override;

  bool WriteHeader(const Header& header);
  bool WriteChannel(const Channel& channel);
  bool AddSingleMessage(const SingleMessage& message);

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
}  // namespace cybertron
}  // namespace apollo

#endif  // CYBERTRON_RECORD_RECORD_FILE_H_
