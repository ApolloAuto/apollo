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

#include "cybertron/tools/cyber_recorder/spliter.h"

namespace apollo {
namespace cybertron {
namespace record {

Spliter::Spliter(const std::string& input_file, const std::string& output_file,
                 bool all_channels, const std::vector<std::string>& channel_vec,
                 uint64_t begin_time, uint64_t end_time)
    : writer_(nullptr),
      input_file_(input_file),
      output_file_(output_file),
      channel_vec_(channel_vec),
      all_channels_(all_channels),
      begin_time_(begin_time),
      end_time_(end_time) {}

Spliter::~Spliter() {}

bool Spliter::Proc() {
  if (begin_time_ > end_time_) {
    AERROR << "begin_time_ larger than end_time_, begin_time_: " << begin_time_
           << "end_time_: " << end_time_;
    return false;
  }
  if (!infileopt_.Open(input_file_)) {
    AERROR << "infileopt_.Open() fail.";
    return false;
  }
  if (!infileopt_.ReadHeader()) {
    AERROR << "infileopt_.ReadHeader() fail.";
    return false;
  }
  Header header = infileopt_.GetHeader();
  if (begin_time_ > header.end_time() || end_time_ < header.begin_time()) {
    AERROR << "time range " << begin_time_ << " to " << end_time_
           << " is not include in this record file.";
    return false;
  }
  writer_.reset(new RecordWriter());
  if (!writer_->Open(output_file_)) {
    AERROR << "writer_->Open() fail.";
    return false;
  }
  infileopt_.ReadIndex();
  Index index = infileopt_.GetIndex();
  std::unordered_map<std::string, int> channel_map;
  for (int i = 0; i < index.indexes_size(); i++) {
    SingleIndex* single_index = index.mutable_indexes(i);
    if (single_index->type() != SectionType::SECTION_CHANNEL) {
      continue;
    }
    ChannelCache* channel_cache = single_index->mutable_channel_cache();
    if (!all_channels_ &&
        std::find(channel_vec_.begin(), channel_vec_.end(),
                  channel_cache->name()) == channel_vec_.end()) {
      continue;
    }
    channel_map[channel_cache->name()] = 0;
    writer_->WriteChannel(channel_cache->name(), channel_cache->message_type(),
                          channel_cache->proto_desc());
  }
  int return_value = true;
  Section section;
  while (infileopt_.ReadSection(&section)) {
    if (section.type == SectionType::SECTION_INDEX) {
      break;
    }
    switch (section.type) {
      case SectionType::SECTION_CHANNEL: {
        Channel channel;
        if (!infileopt_.ReadSection<Channel>(section.size, &channel)) {
          AERROR << "read message fail.";
          return false;
        }
        break;
      }
      case SectionType::SECTION_CHUNK_HEADER: {
        ChunkHeader chunk_header;
        if (!infileopt_.ReadSection<ChunkHeader>(section.size, &chunk_header)) {
          AERROR << "read message fail.";
          return false;
        }
        if (begin_time_ > chunk_header.end_time() ||
            end_time_ < chunk_header.begin_time()) {
          continue;
        }
        if (!SplitChunkBody()) {
          AERROR << "split chunk body fail";
          return true;
        }
        break;
      }
      default: {
        AERROR << "this section should not be here, section type: "
               << section.type;
        break;
      }
    }  // end for switch
  }    // end for while
  AINFO << "split record file done";
  return true;
}  // end for Proc()

bool Spliter::SplitChunkBody() {
  Section section;
  if (!infileopt_.ReadSection(&section)) {
    AERROR << "read section fail.";
    return false;
  }
  ChunkBody chunk_body;
  if (!infileopt_.ReadSection<ChunkBody>(section.size, &chunk_body)) {
    AERROR << "read message fail.";
    return false;
  }
  for (int idx = 0; idx < chunk_body.messages_size(); ++idx) {
    if (!all_channels_ &&
        std::find(channel_vec_.begin(), channel_vec_.end(),
                  chunk_body.messages(idx).channel_name()) ==
            channel_vec_.end()) {
      continue;
    }
    if (chunk_body.messages(idx).time() < begin_time_ ||
        chunk_body.messages(idx).time() > end_time_) {
      continue;
    }
    if (!writer_->WriteMessage(chunk_body.messages(idx))) {
      AERROR << "datafile_->Write() fail.";
      return false;
    }
  }
  return true;
}

}  // namespace record
}  // namespace cybertron
}  // namespace apollo
