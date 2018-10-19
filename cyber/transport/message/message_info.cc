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

#include <arpa/inet.h>

#include "cyber/common/log.h"

namespace apollo {
namespace cyber {
namespace transport {

MessageInfo::MessageInfo() : sender_id_(false), seq_num_(0), spare_id_(false) {}

MessageInfo::MessageInfo(const Identity& sender_id, uint64_t seq_num)
    : sender_id_(sender_id), seq_num_(seq_num), spare_id_(false) {}

MessageInfo::MessageInfo(const Identity& sender_id, uint64_t seq_num,
                         const Identity& spare_id)
    : sender_id_(sender_id), seq_num_(seq_num), spare_id_(spare_id) {}

MessageInfo::MessageInfo(const MessageInfo& another)
    : sender_id_(another.sender_id_),
      seq_num_(another.seq_num_),
      spare_id_(another.spare_id_) {}

MessageInfo::~MessageInfo() {}

MessageInfo& MessageInfo::operator=(const MessageInfo& another) {
  if (this != &another) {
    sender_id_ = another.sender_id_;
    seq_num_ = another.seq_num_;
    spare_id_ = another.spare_id_;
  }
  return *this;
}

bool MessageInfo::operator==(const MessageInfo& another) const {
  if (sender_id_ != another.sender_id_) {
    return false;
  }
  if (seq_num_ != another.seq_num_) {
    return false;
  }
  if (spare_id_ != another.spare_id_) {
    return false;
  }
  return true;
}

bool MessageInfo::SerializeTo(std::string* dst) const {
  RETURN_VAL_IF_NULL(dst, false);

  std::string sender_id_data_str(sender_id_.data(), ID_SIZE);

  // fetch high 4 bytes
  int32_t host_high_seq = (seq_num_ & 0xffffffff00000000) >> 32;
  uint32_t network_high_seq = htonl((uint32_t)host_high_seq);

  // fetch low 4 bytes
  int32_t host_low_seq = seq_num_ & 0x00000000ffffffff;
  uint32_t network_low_seq = htonl((uint32_t)host_low_seq);

  // convert to char array
  char array[8] = {0};
  char* ptr = array;
  memcpy(ptr, reinterpret_cast<char*>(&network_high_seq),
         sizeof(network_high_seq));
  ptr += sizeof(network_high_seq);
  memcpy(ptr, reinterpret_cast<char*>(&network_low_seq),
         sizeof(network_low_seq));

  std::string array_str(array, 8);

  std::string spare_id_data_str(spare_id_.data(), ID_SIZE);

  *dst = sender_id_data_str + array_str + spare_id_data_str;
  return true;
}

bool MessageInfo::DeserializeFrom(const std::string& src) {
  auto given_size = src.size();
  auto target_size = 2 * ID_SIZE + sizeof(seq_num_);
  if (given_size != target_size) {
    AWARN << "src size mismatch, given[" << given_size << "] target["
          << target_size << "]";
    return false;
  }

  std::string sender_id_data_str = src.substr(0, ID_SIZE);
  sender_id_.set_data(sender_id_data_str.data());

  std::string seq_num_str = src.substr(ID_SIZE, sizeof(seq_num_));

  uint32_t network_high_seq = 0;
  char* ptr = const_cast<char*>(seq_num_str.data());
  memcpy(reinterpret_cast<char*>(&network_high_seq), ptr,
         sizeof(network_high_seq));
  ptr += sizeof(network_high_seq);
  int32_t host_high_seq = (int32_t)ntohl(network_high_seq);

  uint32_t network_low_seq = 0;
  memcpy(reinterpret_cast<char*>(&network_low_seq), ptr,
         sizeof(network_low_seq));
  int32_t host_low_seq = (int32_t)ntohl(network_low_seq);

  seq_num_ = ((int64_t)host_high_seq << 32) |
             ((int64_t)host_low_seq & 0x00000000ffffffff);

  std::string spare_id_data_str =
      src.substr(ID_SIZE + sizeof(seq_num_), ID_SIZE);
  spare_id_.set_data(spare_id_data_str.data());

  return true;
}

}  // namespace transport
}  // namespace cyber
}  // namespace apollo
