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

#include "cyber/record/record_viewer.h"

#include <algorithm>

#include "cyber/common/log.h"

namespace apollo {
namespace cyber {
namespace record {

RecordViewer::RecordViewer(const std::shared_ptr<RecordReader>& reader,
                           uint64_t begin_time, uint64_t end_time,
                           const std::set<std::string>& channels)
    : begin_time_(begin_time),
      end_time_(end_time),
      channels_(channels),
      reader_(reader) {
  const auto& header = reader_->header();
  if (begin_time_ < header.begin_time()) {
    begin_time_ = header.begin_time();
  }
  if (end_time_ > header.end_time()) {
    end_time_ = header.end_time();
  }
}

bool RecordViewer::IsValid() const {
  if (begin_time_ > end_time_) {
    AERROR << "Begin time must be earlier than end time"
           << ", begin_time=" << begin_time_ << ", end_time=" << end_time_;
    return false;
  }
  return true;
}

bool RecordViewer::Update(RecordMessage* message) {
  while (reader_->ReadMessage(message, begin_time_, end_time_)) {
    if (channels_.empty() || channels_.count(message->channel_name) == 1) {
      return true;
    }
  }
  return false;
}

uint64_t RecordViewer::begin_time() const { return begin_time_; }

uint64_t RecordViewer::end_time() const { return end_time_; }

std::set<std::string> RecordViewer::GetChannelList() const {
  std::set<std::string> channel_list;
  auto all_channel = reader_->GetChannelList();
  std::set_intersection(all_channel.begin(), all_channel.end(),
                        channels_.begin(), channels_.end(),
                        std::inserter(channel_list, channel_list.end()));
  return channel_list;
}

RecordViewer::Iterator RecordViewer::begin() { return Iterator(this); }

RecordViewer::Iterator RecordViewer::end() { return Iterator(this, true); }

RecordViewer::Iterator::Iterator(RecordViewer* viewer, bool end)
    : end_(end), viewer_(viewer) {
  if (end_) {
    return;
  }
  viewer_->reader_->Reset();
  if (!viewer_->IsValid()) {
    end_ = true;
  } else {
    if (!viewer_->Update(&message_instance_)) {
      end_ = true;
    }
  }
}

bool RecordViewer::Iterator::operator==(Iterator const& other) const {
  if (other.end_) {
    return end_;
  }
  return index_ == other.index_ && viewer_ == other.viewer_;
}

bool RecordViewer::Iterator::operator!=(const Iterator& rhs) const {
  return !(*this == rhs);
}

void RecordViewer::Iterator::operator++() {
  index_++;
  if (!viewer_->Update(&message_instance_)) {
    end_ = true;
  }
}

RecordViewer::Iterator::pointer RecordViewer::Iterator::operator->() {
  return &message_instance_;
}

RecordViewer::Iterator::reference RecordViewer::Iterator::operator*() {
  return message_instance_;
}

}  // namespace record
}  // namespace cyber
}  // namespace apollo
