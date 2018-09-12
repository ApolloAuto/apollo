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

#include "cybertron/record/record_reader.h"

namespace apollo {
namespace cybertron {
namespace record {

RecordReader::RecordReader() {}

RecordReader::~RecordReader() {
  if (load_thread_ && load_thread_->joinable()) {
    load_thread_->join();
  }
  Close();
}

bool RecordReader::Open(const std::string& file, uint64_t begin_time,
                        uint64_t end_time) {
  file_ = file;
  path_ = file_;
  begin_time_ = begin_time;
  end_time_ = end_time;
  file_reader_.reset(new RecordFileReader());
  current_message_.reset(new RecordMessage());

  if (!file_reader_->Open(path_)) {
    AERROR << "open record file failed. file: " << path_;
    return false;
  }
  header_ = file_reader_->GetHeader();
  if (begin_time_ < header_.begin_time()) {
    begin_time_ = header_.begin_time();
  }
  if (end_time_ > header_.end_time()) {
    end_time_ = header_.end_time();
  }
  if (begin_time_ > end_time_) {
    AERROR << "begin time is larger than end time"
           << ", begin_time_=" << begin_time_ << ", end_time_=" << end_time_;
    return false;
  }
  if (!InitLoadThread()) {
    AERROR << "init loader error.";
    return false;
  }
  is_reading_ = true;
  return true;
}

void RecordReader::Close() {
  if (is_reading_) {
    file_reader_->Close();
    is_reading_ = false;
  }
}

bool RecordReader::InitLoadThread() {
  auto f = std::bind(&RecordReader::LoadChunk, this, std::placeholders::_1,
                     std::placeholders::_2);
  load_thread_ = std::make_shared<std::thread>(f, begin_time_, end_time_);
  if (load_thread_ == nullptr) {
    AERROR << "init loader thread error.";
    return false;
  }
  return true;
}

bool RecordReader::ReadMessage() {
  std::lock_guard<std::mutex> lck(mutex_);
  if (!message_queue_.empty()) {
    current_message_ = message_queue_.front();
    message_queue_.pop();
    return true;
  }
  if (!looped_readback_ && loaded_all_message_) {
    AINFO << "reach the last message";
  } else {
    AERROR << "read too fast, wait for load thread";
  }
  return false;
}

bool RecordReader::EndOfFile() {
  std::lock_guard<std::mutex> lck(mutex_);
  if (message_queue_.empty() && !looped_readback_ && loaded_all_message_) {
    return true;
  }
  return false;
}

void RecordReader::LoadChunk(uint64_t from_time, uint64_t to_time) {
  uint64_t loop_time = end_time_ - begin_time_;
  uint64_t total_message_number = header_.message_number();
  if (total_message_number <= 0) {
    AERROR << "total message number is zero.";
    return;
  }
  uint64_t average_period = average_period = loop_time / total_message_number;
  if (average_period < 1e7) {
    average_period = 1e7;  // 10ms
  }
  double average_hz = total_message_number / (loop_time * 1e-9);
  uint32_t preload_queue_size = (uint32_t)average_hz * preload_seconds_;
  if (preload_queue_size < min_queue_size_) {
    preload_queue_size = min_queue_size_;
  }
  uint32_t loop_num = 0;
  do {
    AINFO << "new loop started";
    bool skip_next_chunk_body(false);
    file_reader_->ReadHeader();
    while (!file_reader_->EndOfFile()) {
      while (message_queue_.size() >= preload_queue_size) {
        std::this_thread::sleep_for(std::chrono::nanoseconds(average_period));
      }
      Section section;
      if (!file_reader_->ReadSection(&section)) {
        AERROR << "read section fail";
        return;
      }
      if (section.type == SectionType::SECTION_INDEX) {
        break;
      }
      switch (section.type) {
        case SectionType::SECTION_CHANNEL: {
          Channel chan;
          if (!file_reader_->ReadSection<Channel>(section.size, &chan)) {
            AERROR << "read channel section fail.";
            return;
          }
          OnNewChannel(chan.name(), chan.message_type(), chan.proto_desc());
          break;
        }
        case SectionType::SECTION_CHUNK_HEADER: {
          ChunkHeader chdr;
          if (!file_reader_->ReadSection<ChunkHeader>(section.size, &chdr)) {
            AERROR << "read chunk header section fail.";
            return;
          }
          if (from_time > chdr.end_time() || to_time < chdr.begin_time()) {
            skip_next_chunk_body = true;
          }
          break;
        }
        case SectionType::SECTION_CHUNK_BODY: {
          if (skip_next_chunk_body) {
            file_reader_->SkipSection(section.size);
            skip_next_chunk_body = false;
            break;
          }
          ChunkBody cbd;
          if (!file_reader_->ReadSection<ChunkBody>(section.size, &cbd)) {
            AERROR << "read chunk body section fail.";
            return;
          }
          for (int idx = 0; idx < cbd.messages_size(); ++idx) {
            uint64_t time(cbd.messages(idx).time());
            if (time < from_time || time > to_time) {
              continue;
            }
            std::string channel_name(cbd.messages(idx).channel_name());
            std::shared_ptr<RawMessage> raw_message(
                new RawMessage(cbd.messages(idx).content()));
            std::shared_ptr<RecordMessage> message_queue_item(new RecordMessage(
                channel_name, raw_message, cbd.messages(idx).time()));
            OnNewMessage(channel_name);
            {
              std::lock_guard<std::mutex> lck(mutex_);
              message_queue_.push(message_queue_item);
            }
          }
          break;
        }
        default: {
          AERROR << "section should not be here, section type: "
                 << section.type;
          break;
        }
      }  // end switch for section type
    }    // end while, condition is not EOF
    loop_num++;
  } while (looped_readback_);
  loaded_all_message_ = true;
}

const std::string& RecordReader::CurrentMessageChannelName() {
  std::lock_guard<std::mutex> lck(mutex_);
  return current_message_->channel_name;
}

uint64_t RecordReader::CurrentMessageTime() {
  std::lock_guard<std::mutex> lck(mutex_);
  return current_message_->time;
}

std::shared_ptr<RawMessage> RecordReader::CurrentRawMessage() {
  std::lock_guard<std::mutex> lck(mutex_);
  return current_message_->raw_message;
}

}  // namespace record
}  // namespace cybertron
}  // namespace apollo
