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

#include "cybertron/record/record_base.h"

namespace apollo {
namespace cybertron {
namespace record {

RecordBase::RecordBase() {}

RecordBase::~RecordBase() {}

const Header RecordBase::GetHeader() const { return header_; }

void RecordBase::OnNewChannel(const std::string& channel_name,
                              const std::string& message_type,
                              const std::string& proto_desc) {
  auto search = channel_message_number_map_.find(channel_name);
  if (search == channel_message_number_map_.end()) {
    channel_message_number_map_[channel_name] = 0;
  }
  channel_message_type_map_[channel_name] = message_type;
  channel_proto_desc_map_[channel_name] = proto_desc;
}

void RecordBase::OnNewMessage(const std::string& channel_name) {
  auto search = channel_message_number_map_.find(channel_name);
  if (search != channel_message_number_map_.end()) {
    channel_message_number_map_[channel_name]++;
  }
}

uint64_t RecordBase::GetMessageNumber(const std::string& channel_name) const {
  auto search = channel_message_number_map_.find(channel_name);
  if (search != channel_message_number_map_.end()) {
    return search->second;
  }
  return 0;
}

const std::string& RecordBase::GetMessageType(
    const std::string& channel_name) const {
  auto search = channel_message_type_map_.find(channel_name);
  if (search != channel_message_type_map_.end()) {
    return search->second;
  }
  return "";
}

const std::string& RecordBase::GetProtoDesc(
    const std::string& channel_name) const {
  auto search = channel_proto_desc_map_.find(channel_name);
  if (search != channel_proto_desc_map_.end()) {
    return search->second;
  }
  return "";
}

}  // namespace record
}  // namespace cybertron
}  // namespace apollo
