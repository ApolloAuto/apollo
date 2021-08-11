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

#ifndef CYBER_TRANSPORT_MESSAGE_MESSAGE_INFO_H_
#define CYBER_TRANSPORT_MESSAGE_MESSAGE_INFO_H_

#include <cstddef>
#include <cstdint>
#include <string>

#include "cyber/transport/common/identity.h"

namespace apollo {
namespace cyber {
namespace transport {

class MessageInfo {
 public:
  MessageInfo();
  MessageInfo(const Identity& sender_id, uint64_t seq_num);
  MessageInfo(const Identity& sender_id, uint64_t seq_num,
              const Identity& spare_id);
  MessageInfo(const MessageInfo& another);
  virtual ~MessageInfo();

  MessageInfo& operator=(const MessageInfo& another);
  bool operator==(const MessageInfo& another) const;
  bool operator!=(const MessageInfo& another) const;

  bool SerializeTo(std::string* dst) const;
  bool SerializeTo(char* dst, std::size_t len) const;
  bool DeserializeFrom(const std::string& src);
  bool DeserializeFrom(const char* src, std::size_t len);

  // getter and setter
  const Identity& sender_id() const { return sender_id_; }
  void set_sender_id(const Identity& sender_id) { sender_id_ = sender_id; }

  uint64_t channel_id() const { return channel_id_; }
  void set_channel_id(uint64_t channel_id) { channel_id_ = channel_id; }

  uint64_t seq_num() const { return seq_num_; }
  void set_seq_num(uint64_t seq_num) { seq_num_ = seq_num; }

  const Identity& spare_id() const { return spare_id_; }
  void set_spare_id(const Identity& spare_id) { spare_id_ = spare_id; }

  static const std::size_t kSize;

 private:
  Identity sender_id_;
  uint64_t channel_id_ = 0;
  uint64_t seq_num_ = 0;
  Identity spare_id_;
};

}  // namespace transport
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_TRANSPORT_MESSAGE_MESSAGE_INFO_H_
