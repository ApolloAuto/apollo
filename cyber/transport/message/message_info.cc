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

#include "cyber/transport/message/message_info.h"

#include <cstring>

#include "cyber/common/log.h"

namespace apollo {
namespace cyber {
namespace transport {

const std::size_t MessageInfo::kSize = 2 * ID_SIZE + sizeof(uint64_t) + \
                                        sizeof(uint64_t) + sizeof(int32_t) + \
                                        sizeof(uint64_t);

MessageInfo::MessageInfo() : sender_id_(false), spare_id_(false) {}

MessageInfo::MessageInfo(const Identity& sender_id, uint64_t seq_num)
    : sender_id_(sender_id), seq_num_(seq_num), spare_id_(false) {}

MessageInfo::MessageInfo(const Identity& sender_id, uint64_t seq_num,
                         const Identity& spare_id)
    : sender_id_(sender_id), seq_num_(seq_num), spare_id_(spare_id) {}

MessageInfo::MessageInfo(const MessageInfo& another)
    : sender_id_(another.sender_id_),
      channel_id_(another.channel_id_),
      seq_num_(another.seq_num_),
      spare_id_(another.spare_id_) {}

MessageInfo::~MessageInfo() {}

MessageInfo& MessageInfo::operator=(const MessageInfo& another) {
  if (this != &another) {
    sender_id_ = another.sender_id_;
    channel_id_ = another.channel_id_;
    seq_num_ = another.seq_num_;
    spare_id_ = another.spare_id_;
  }
  return *this;
}

bool MessageInfo::operator==(const MessageInfo& another) const {
  return sender_id_ == another.sender_id_ &&
         channel_id_ == another.channel_id_ && seq_num_ == another.seq_num_ &&
         spare_id_ == another.spare_id_;
}

bool MessageInfo::operator!=(const MessageInfo& another) const {
  return !(*this == another);
}

bool MessageInfo::SerializeTo(std::string* dst) const {
  RETURN_VAL_IF_NULL(dst, false);

  dst->assign(sender_id_.data(), ID_SIZE);
  dst->append(
    reinterpret_cast<const char*>(&channel_id_), sizeof(channel_id_));
  dst->append(
    reinterpret_cast<const char*>(&seq_num_), sizeof(seq_num_));
  dst->append(spare_id_.data(), ID_SIZE);
  dst->append(reinterpret_cast<const char*>(
    &msg_seq_num_), sizeof(msg_seq_num_));
  dst->append(reinterpret_cast<const char*>(
    &send_time_), sizeof(send_time_));
  return true;
}

bool MessageInfo::SerializeTo(char* dst, std::size_t len) const {
  if (dst == nullptr || len < kSize) {
    return false;
  }

  char* ptr = dst;
  std::memcpy(ptr, sender_id_.data(), ID_SIZE);
  ptr += ID_SIZE;
  std::memcpy(ptr,
    reinterpret_cast<const char*>(&channel_id_), sizeof(channel_id_));
  ptr += sizeof(channel_id_);
  std::memcpy(ptr,
    reinterpret_cast<const char*>(&seq_num_), sizeof(seq_num_));
  ptr += sizeof(seq_num_);
  std::memcpy(ptr, spare_id_.data(), ID_SIZE);
  ptr += ID_SIZE;
  std::memcpy(ptr,
    reinterpret_cast<const char*>(&msg_seq_num_), sizeof(msg_seq_num_));
  ptr += sizeof(msg_seq_num_);
  std::memcpy(ptr,
    reinterpret_cast<const char*>(&send_time_), sizeof(send_time_));
  return true;
}

bool MessageInfo::DeserializeFrom(const std::string& src) {
  return DeserializeFrom(src.data(), src.size());
}

bool MessageInfo::DeserializeFrom(const char* src, std::size_t len) {
  RETURN_VAL_IF_NULL(src, false);
  if (len != kSize) {
    AWARN << "src size mismatch, given[" << len << "] target[" << kSize << "]";
    return false;
  }

  char* ptr = const_cast<char*>(src);
  sender_id_.set_data(ptr);
  ptr += ID_SIZE;
  std::memcpy(
    reinterpret_cast<char*>(&channel_id_), ptr, sizeof(channel_id_));
  ptr += sizeof(channel_id_);
  std::memcpy(
    reinterpret_cast<char*>(&seq_num_), ptr, sizeof(seq_num_));
  ptr += sizeof(seq_num_);
  spare_id_.set_data(ptr);
  ptr += ID_SIZE;
  std::memcpy(
    reinterpret_cast<char*>(&msg_seq_num_), ptr, sizeof(msg_seq_num_));
  ptr += sizeof(msg_seq_num_);
  std::memcpy(
    reinterpret_cast<char*>(&send_time_), ptr, sizeof(send_time_));
  return true;
}

}  // namespace transport
}  // namespace cyber
}  // namespace apollo
