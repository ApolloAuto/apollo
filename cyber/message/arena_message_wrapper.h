/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
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
#ifndef CYBER_MESSAGE_ARENA_MESSAGE_WRAPPER_H_
#define CYBER_MESSAGE_ARENA_MESSAGE_WRAPPER_H_

#include <cstdint>
#include <cstring>
#include <memory>

#include "cyber/message/arena_manager_base.h"

namespace apollo {
namespace cyber {
namespace message {

union ArenaMessageWrapperMeta {
  struct {
    uint64_t version_;
    uint64_t addr_offset_;
  } struct_;
  uint8_t bytes_[128];
};

union ArenaMessageWrapperExtended {
  uint8_t bytes_[256];
};

union ArenaMessageWrapperDataStruct {
  struct {
    union ArenaMessageWrapperMeta meta_;
    union ArenaMessageWrapperExtended extended_;
  } struct_;
  uint8_t bytes_[1024];
};

class ArenaMessageWrapper {
 public:
  static const uint64_t kDefaultVersion = 1L;

  ArenaMessageWrapper() : arena_manager_(nullptr) {
    data_.struct_.meta_.struct_.version_ = kDefaultVersion;
    data_.struct_.meta_.struct_.addr_offset_ = 0;
  }

  explicit ArenaMessageWrapper(ArenaManagerBase* arena_manager)
      : arena_manager_(arena_manager) {
    data_.struct_.meta_.struct_.version_ = kDefaultVersion;
    data_.struct_.meta_.struct_.addr_offset_ = 0;
  }

  virtual ~ArenaMessageWrapper() {}

  void SetVersion(uint64_t version) {
    data_.struct_.meta_.struct_.version_ = version;
  }
  uint64_t GetVersion() const { return data_.struct_.meta_.struct_.version_; }

  uint64_t GetMessageAddress() const {
    return arena_manager_->GetBaseAddress(this) +
           data_.struct_.meta_.struct_.addr_offset_;
  }

  void* GetData() { return reinterpret_cast<void*>(data_.bytes_); }

  bool FillMeta(void* meta, uint64_t size) {
    if (size > 128) {
      return false;
    }
    memcpy(&data_.struct_.meta_, meta, size);
    return true;
  }

  ArenaMessageWrapperMeta* GetMeta() {
    return reinterpret_cast<ArenaMessageWrapperMeta*>(
        data_.struct_.meta_.bytes_);
  }

  bool FillExtended(void* extended, uint64_t size) {
    if (size > 256) {
      return false;
    }
    memcpy(&data_.struct_.extended_, extended, size);
    return true;
  }

  template <typename T>
  T* GetExtended() {
    return reinterpret_cast<T*>(data_.struct_.extended_.bytes_);
  }

  template <typename MessageT>
  MessageT* GetMessage() {
    // uint64_t base_address =
    //     reinterpret_cast<uint64_t>(arena_manager_->GetBaseAddress());
    // uint64_t message_address = base_address + data_.struct_.addr_offset_;
    // return std::shared_ptr<MessageT>(
    //     reinterpret_cast<MessageT*>(GetMessageAddress()));
    auto msg_ptr = arena_manager_->GetMessage<MessageT>(this);
    return msg_ptr;
  }

  template <typename MessageT>
  MessageT* SetMessage(const MessageT& message) {
    auto msg_ptr = arena_manager_->SetMessage(this, message);
    return msg_ptr;
  }

 private:
  // union {
  //   struct {
  //     union {
  //       uint64_t version_;
  //       uint64_t addr_offset_;
  //       uint8_t bytes_[128];
  //     } meta_;
  //     union {
  //       uint8_t bytes_[256];
  //     } extended_;
  //   } struct_;
  //   uint8_t bytes_[1024];
  // } data_;
  ArenaMessageWrapperDataStruct data_;
  // message holder
  std::shared_ptr<void*> message_;
  ArenaManagerBase* arena_manager_;
};

}  // namespace message
}  // namespace cyber
}  // namespace apollo

#endif
