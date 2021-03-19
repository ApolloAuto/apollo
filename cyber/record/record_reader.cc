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

#include "cyber/record/record_reader.h"

#include <utility>

namespace apollo {
namespace cyber {
namespace record {

using apollo::cyber::proto::Channel;
using apollo::cyber::proto::ChunkBody;
using apollo::cyber::proto::ChunkHeader;
using apollo::cyber::proto::SectionType;

RecordReader::~RecordReader() {}

RecordReader::RecordReader(const std::string& file) {
  file_reader_.reset(new RecordFileReader());
  if (!file_reader_->Open(file)) {
    AERROR << "Failed to open record file: " << file;
    return;
  }
  chunk_.reset(new ChunkBody());
  is_valid_ = true;
  header_ = file_reader_->GetHeader();
  if (file_reader_->ReadIndex()) {
    index_ = file_reader_->GetIndex();
    for (int i = 0; i < index_.indexes_size(); ++i) {
      auto single_idx = index_.mutable_indexes(i);
      if (single_idx->type() != SectionType::SECTION_CHANNEL) {
        continue;
      }
      if (!single_idx->has_channel_cache()) {
        AERROR << "Single channel index does not have channel_cache.";
        continue;
      }
      auto channel_cache = single_idx->mutable_channel_cache();
      channel_info_.insert(
          std::make_pair(channel_cache->name(), *channel_cache));
    }
  }
  file_reader_->Reset();
}

void RecordReader::Reset() {
  file_reader_->Reset();
  reach_end_ = false;
  message_index_ = 0;
  chunk_.reset(new ChunkBody());
}

std::set<std::string> RecordReader::GetChannelList() const {
  std::set<std::string> channel_list;
  for (auto& item : channel_info_) {
    channel_list.insert(item.first);
  }
  return channel_list;
}

bool RecordReader::ReadMessage(RecordMessage* message, uint64_t begin_time,
                               uint64_t end_time) {
  if (!is_valid_) {
    return false;
  }

  if (begin_time > header_.end_time() || end_time < header_.begin_time()) {
    return false;
  }

  while (message_index_ < chunk_->messages_size()) {
    const auto& next_message = chunk_->messages(message_index_);
    uint64_t time = next_message.time();
    if (time > end_time) {
      return false;
    }
    ++message_index_;
    if (time < begin_time) {
      continue;
    }

    message->channel_name = next_message.channel_name();
    message->content = next_message.content();
    message->time = time;
    return true;
  }

  ADEBUG << "Read next chunk.";
  if (ReadNextChunk(begin_time, end_time)) {
    ADEBUG << "Read chunk successfully.";
    message_index_ = 0;
    return ReadMessage(message, begin_time, end_time);
  }
  ADEBUG << "No chunk to read.";
  return false;
}

bool RecordReader::ReadNextChunk(uint64_t begin_time, uint64_t end_time) {
  bool skip_next_chunk_body = false;
  while (!reach_end_) {
    Section section;
    if (!file_reader_->ReadSection(&section)) {
      AERROR << "Failed to read section, file: " << file_reader_->GetPath();
      return false;
    }
    switch (section.type) {
      case SectionType::SECTION_INDEX: {
        file_reader_->SkipSection(section.size);
        reach_end_ = true;
        break;
      }
      case SectionType::SECTION_CHANNEL: {
        ADEBUG << "Read channel section of size: " << section.size;
        Channel channel;
        if (!file_reader_->ReadSection<Channel>(section.size, &channel)) {
          AERROR << "Failed to read channel section.";
          return false;
        }
        break;
      }
      case SectionType::SECTION_CHUNK_HEADER: {
        ADEBUG << "Read chunk header section of size: " << section.size;
        ChunkHeader header;
        if (!file_reader_->ReadSection<ChunkHeader>(section.size, &header)) {
          AERROR << "Failed to read chunk header section.";
          return false;
        }
        if (header.end_time() < begin_time) {
          skip_next_chunk_body = true;
        }
        if (header.begin_time() > end_time) {
          return false;
        }
        break;
      }
      case SectionType::SECTION_CHUNK_BODY: {
        if (skip_next_chunk_body) {
          file_reader_->SkipSection(section.size);
          skip_next_chunk_body = false;
          break;
        }

        chunk_.reset(new ChunkBody());
        if (!file_reader_->ReadSection<ChunkBody>(section.size, chunk_.get())) {
          AERROR << "Failed to read chunk body section.";
          return false;
        }
        return true;
      }
      default: {
        AERROR << "Invalid section, type: " << section.type
               << ", size: " << section.size;
        return false;
      }
    }
  }
  return false;
}

uint64_t RecordReader::GetMessageNumber(const std::string& channel_name) const {
  auto search = channel_info_.find(channel_name);
  if (search == channel_info_.end()) {
    return 0;
  }
  return search->second.message_number();
}

const std::string& RecordReader::GetMessageType(
    const std::string& channel_name) const {
  auto search = channel_info_.find(channel_name);
  if (search == channel_info_.end()) {
    return kEmptyString;
  }
  return search->second.message_type();
}

const std::string& RecordReader::GetProtoDesc(
    const std::string& channel_name) const {
  auto search = channel_info_.find(channel_name);
  if (search == channel_info_.end()) {
    return kEmptyString;
  }
  return search->second.proto_desc();
}

}  // namespace record
}  // namespace cyber
}  // namespace apollo
