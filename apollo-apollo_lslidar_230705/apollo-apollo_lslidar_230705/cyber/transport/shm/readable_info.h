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

#ifndef CYBER_TRANSPORT_SHM_READABLE_INFO_H_
#define CYBER_TRANSPORT_SHM_READABLE_INFO_H_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace apollo {
namespace cyber {
namespace transport {

class ReadableInfo;
using ReadableInfoPtr = std::shared_ptr<ReadableInfo>;

class ReadableInfo {
 public:
  ReadableInfo();
  ReadableInfo(uint64_t host_id, uint32_t block_index, uint64_t channel_id);
  virtual ~ReadableInfo();

  ReadableInfo& operator=(const ReadableInfo& other);

  bool DeserializeFrom(const std::string& src);
  bool DeserializeFrom(const char* src, std::size_t len);
  bool SerializeTo(std::string* dst) const;

  uint64_t host_id() const { return host_id_; }
  void set_host_id(uint64_t host_id) { host_id_ = host_id; }

  uint32_t block_index() const { return block_index_; }
  void set_block_index(uint32_t block_index) { block_index_ = block_index; }

  uint64_t channel_id() const { return channel_id_; }
  void set_channel_id(uint64_t channel_id) { channel_id_ = channel_id; }

  static const size_t kSize;

 private:
  uint64_t host_id_;
  uint32_t block_index_;
  uint64_t channel_id_;
};

}  // namespace transport
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_TRANSPORT_SHM_READABLE_INFO_H_
