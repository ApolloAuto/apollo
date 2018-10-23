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

#include "cyber/record/file/record_file_writer.h"

#include "cyber/common/file.h"

namespace apollo {
namespace cyber {
namespace record {

RecordFileWriter::RecordFileWriter() {}

RecordFileWriter::~RecordFileWriter() { Close(); }

bool RecordFileWriter::Open(const std::string& path) {
  std::lock_guard<std::mutex> lock(mutex_);
  path_ = path;
  std::ios_base::openmode mode = std::ios::binary | std::ios::out;
  ofstream_.open(path, mode);
  if (!ofstream_.is_open()) {
    AERROR << "file [" << path << "] open error.";
    return false;
  }
  chunk_active_.reset(new Chunk());
  chunk_flush_.reset(new Chunk());
  is_writing_ = true;

  // start flush thread
  flush_thread_ = std::make_shared<std::thread>([this]() { this->Flush(); });
  if (flush_thread_ == nullptr) {
    AERROR << "init flush thread error.";
    return false;
  }
  ADEBUG << "record file opened, file: " << path_;
  return true;
}

void RecordFileWriter::Close() {
  if (!ofstream_.is_open()) {
    return;
  }

  if (is_writing_) {
    std::lock_guard<std::recursive_mutex> lock(chunk_mutex_);
    // wait for the flush operation that may exist now
    while (!chunk_flush_->empty()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    {
      std::unique_lock<std::mutex> flush_lock(flush_mutex_);
      chunk_flush_.swap(chunk_active_);
      flush_cv_.notify_one();
    }
    while (!chunk_flush_->empty()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    // chunk_active_->clear();
    is_writing_ = false;
    flush_cv_.notify_all();
  }

  if (flush_thread_ && flush_thread_->joinable()) {
    flush_thread_->join();
    flush_thread_ = nullptr;
  }

  try {
    RefreshIndex();
    WriteIndex(index_);
    header_.set_is_complete(true);
    WriteHeader(header_);
  } catch (std::exception& e) {
    AERROR << "record file closed with exception : " << e.what();
  }

  ofstream_.close();
  ADEBUG << "record file closed, file: " << path_;
}

void RecordFileWriter::RefreshIndex() {
  for (int i = 0; i < index_.indexes_size(); i++) {
    SingleIndex* single_index = index_.mutable_indexes(i);
    if (single_index->type() == SectionType::SECTION_CHANNEL) {
      if (!single_index->has_channel_cache()) {
        AERROR << "single channel index do not have channel_cache field.";
        continue;
      }
      ChannelCache* channel_cache = single_index->mutable_channel_cache();
      if (channel_message_number_map_.find(channel_cache->name()) !=
          channel_message_number_map_.end()) {
        channel_cache->set_message_number(
            channel_message_number_map_[channel_cache->name()]);
      }
    }
  }
}

bool RecordFileWriter::WriteHeader(const Header& header) {
  std::lock_guard<std::mutex> lock(mutex_);
  header_ = header;
  ofstream_.seekp(0, std::ios::beg);
  if (!WriteSection<Header>(SectionType::SECTION_HEADER, header_,
                            HEADER_LENGTH)) {
    AERROR << "write section fail";
    return false;
  }
  return true;
}

bool RecordFileWriter::WriteIndex(const Index& index) {
  std::lock_guard<std::mutex> lock(mutex_);

  // update header
  header_.set_index_position(ofstream_.tellp());

  if (!WriteSection<Index>(SectionType::SECTION_INDEX, index, 0)) {
    AERROR << "write section fail";
    return false;
  }

  // update header
  header_.set_size(ofstream_.tellp());
  return true;
}

bool RecordFileWriter::WriteChannel(const Channel& channel) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!WriteSection<Channel>(SectionType::SECTION_CHANNEL, channel, 0)) {
    AERROR << "write section fail";
    return false;
  }

  // update header
  header_.set_size(ofstream_.tellp());
  header_.set_channel_number(header_.channel_number() + 1);

  // update index
  SingleIndex* single_index = index_.add_indexes();
  single_index->set_type(SectionType::SECTION_CHANNEL);
  single_index->set_position(ofstream_.tellp());
  ChannelCache* channel_cache = new ChannelCache();
  channel_cache->set_name(channel.name());
  channel_cache->set_message_number(0);
  channel_cache->set_message_type(channel.message_type());
  channel_cache->set_proto_desc(channel.proto_desc());
  single_index->set_allocated_channel_cache(channel_cache);

  return true;
}

bool RecordFileWriter::WriteChunk(const ChunkHeader& chunk_header,
                                  const ChunkBody& chunk_body) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!WriteSection<ChunkHeader>(SectionType::SECTION_CHUNK_HEADER,
                                 chunk_header, 0)) {
    AERROR << "write chunk header fail";
    return false;
  }

  if (!WriteSection<ChunkBody>(SectionType::SECTION_CHUNK_BODY, chunk_body,
                               0)) {
    AERROR << "write chunk body fail";
    return false;
  }

  // update header
  header_.set_chunk_number(header_.chunk_number() + 1);
  if (header_.begin_time() == 0) {
    header_.set_begin_time(chunk_header.begin_time());
  }
  header_.set_end_time(chunk_header.end_time());
  header_.set_message_number(header_.message_number() +
                             chunk_header.message_number());

  // update index
  SingleIndex* single_index = index_.add_indexes();
  single_index->set_type(SectionType::SECTION_CHUNK_HEADER);
  single_index->set_position(ofstream_.tellp());
  ChunkHeaderCache* chunk_header_cache = new ChunkHeaderCache();
  chunk_header_cache->set_begin_time(chunk_header.begin_time());
  chunk_header_cache->set_end_time(chunk_header.end_time());
  chunk_header_cache->set_message_number(chunk_header.message_number());
  chunk_header_cache->set_raw_size(chunk_header.raw_size());
  single_index->set_allocated_chunk_header_cache(chunk_header_cache);

  // update index
  single_index = index_.add_indexes();
  single_index->set_type(SectionType::SECTION_CHUNK_BODY);
  single_index->set_position(ofstream_.tellp());
  ChunkBodyCache* chunk_body_cache = new ChunkBodyCache();
  chunk_body_cache->set_message_number(chunk_body.messages_size());
  single_index->set_allocated_chunk_body_cache(chunk_body_cache);

  return true;
}

bool RecordFileWriter::WriteMessage(const SingleMessage& message) {
  std::lock_guard<std::recursive_mutex> lock(chunk_mutex_);

  chunk_active_->add(message);

  std::string channel_name = message.channel_name();
  auto it = channel_message_number_map_.find(channel_name);
  if (it != channel_message_number_map_.end()) {
    it->second++;
  } else {
    channel_message_number_map_.insert(std::make_pair(channel_name, 1));
  }

  bool need_flush = false;

  if (header_.chunk_interval() > 0 &&
      message.time() - chunk_active_->header_.begin_time() >
          header_.chunk_interval()) {
    need_flush = true;
  }

  if (header_.chunk_raw_size() > 0 &&
      chunk_active_->header_.raw_size() > header_.chunk_raw_size()) {
    need_flush = true;
  }

  if (!need_flush) {
    return true;
  }

  {
    std::unique_lock<std::mutex> flush_lock(flush_mutex_);
    chunk_flush_.swap(chunk_active_);
    flush_cv_.notify_one();
  }

  chunk_active_->clear();
  return true;
}

void RecordFileWriter::Flush() {
  while (is_writing_) {
    std::unique_lock<std::mutex> flush_lock(flush_mutex_);
    flush_cv_.wait(flush_lock,
                   [this] { return !chunk_flush_->empty() || !is_writing_; });
    if (!is_writing_) {
      break;
    }
    if (chunk_flush_->empty()) {
      continue;
    }
    if (!WriteChunk(chunk_flush_->header_, chunk_flush_->body_)) {
      AERROR << "write chunk fail.";
    }
    chunk_flush_->clear();
  }
}

uint64_t RecordFileWriter::GetMessageNumber(
    const std::string& channel_name) const {
  auto search = channel_message_number_map_.find(channel_name);
  if (search != channel_message_number_map_.end()) {
    return search->second;
  }
  return 0;
}

}  // namespace record
}  // namespace cyber
}  // namespace apollo
