/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#ifndef CYBER_MESSAGE_MESSAGE_HEADER_H_
#define CYBER_MESSAGE_MESSAGE_HEADER_H_

#include <arpa/inet.h>

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <string>

namespace apollo {
namespace cyber {
namespace message {

class MessageHeader {
 public:
  MessageHeader() {
    reset_magic_num();
    reset_seq();
    reset_timestamp_ns();
    reset_src_id();
    reset_dst_id();
    reset_msg_type();
    reset_res();
    reset_content_size();
  }

  bool is_magic_num_match(const char* other, size_t other_len) const {
    if (other == nullptr || other_len != sizeof(magic_num_)) {
      return false;
    }
    return memcmp(magic_num_, other, sizeof(magic_num_)) == 0;
  }
  void reset_magic_num() { memcpy(magic_num_, "BDACBDAC", sizeof(magic_num_)); }

  uint64_t seq() const { return ConvertArrayTo64(seq_); }
  void set_seq(uint64_t seq) { Convert64ToArray(seq, const_cast<char*>(seq_)); }
  void reset_seq() { memset(seq_, 0, sizeof(seq_)); }

  uint64_t timestamp_ns() const { return ConvertArrayTo64(timestamp_ns_); }
  void set_timestamp_ns(uint64_t timestamp_ns) {
    Convert64ToArray(timestamp_ns, const_cast<char*>(timestamp_ns_));
  }
  void reset_timestamp_ns() { memset(timestamp_ns_, 0, sizeof(timestamp_ns_)); }

  uint64_t src_id() const { return ConvertArrayTo64(src_id_); }
  void set_src_id(uint64_t src_id) {
    Convert64ToArray(src_id, const_cast<char*>(src_id_));
  }
  void reset_src_id() { memset(src_id_, 0, sizeof(src_id_)); }

  uint64_t dst_id() const { return ConvertArrayTo64(dst_id_); }
  void set_dst_id(uint64_t dst_id) {
    Convert64ToArray(dst_id, const_cast<char*>(dst_id_));
  }
  void reset_dst_id() { memset(dst_id_, 0, sizeof(dst_id_)); }

  const char* msg_type() const { return msg_type_; }
  void set_msg_type(const char* msg_type, size_t msg_type_len) {
    if (msg_type == nullptr || msg_type_len == 0) {
      return;
    }
    size_t real_len = msg_type_len;
    if (msg_type_len >= sizeof(msg_type_)) {
      real_len = sizeof(msg_type_) - 1;
    }
    reset_msg_type();
    memcpy(msg_type_, msg_type, real_len);
  }
  void reset_msg_type() { memset(msg_type_, 0, sizeof(msg_type_)); }

  void reset_res() { memset(res_, 0, sizeof(res_)); }

  uint32_t content_size() const { return ConvertArrayTo32(content_size_); }
  void set_content_size(uint32_t content_size) {
    Convert32ToArray(content_size, const_cast<char*>(content_size_));
  }
  void reset_content_size() { memset(content_size_, 0, sizeof(content_size_)); }

 private:
  void Convert32ToArray(uint32_t input, char* output) {
    uint32_t n = htonl(input);
    memcpy(static_cast<void*>(output), static_cast<const void*>(&n), sizeof(n));
  }

  void Convert64ToArray(uint64_t input, char* output) {
    uint32_t h_high =
        static_cast<uint32_t>((input & 0xffffffff00000000UL) >> 32);
    uint32_t h_low = static_cast<uint32_t>(input & 0x00000000ffffffffUL);
    Convert32ToArray(h_high, output);
    Convert32ToArray(h_low, output + 4);
  }

  uint32_t ConvertArrayTo32(const char* input) const {
    uint32_t n = 0;
    memcpy(static_cast<void*>(&n), static_cast<const void*>(input), sizeof(n));
    return ntohl(n);
  }

  uint64_t ConvertArrayTo64(const char* input) const {
    uint64_t high = ConvertArrayTo32(input);
    uint64_t low = ConvertArrayTo32(input + 4);
    return (high << 32) | low;
  }

  char magic_num_[8];
  char seq_[8];
  char timestamp_ns_[8];
  char src_id_[8];
  char dst_id_[8];
  char msg_type_[129];
  char res_[19];
  char content_size_[4];
};

}  // namespace message
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_MESSAGE_MESSAGE_HEADER_H_
