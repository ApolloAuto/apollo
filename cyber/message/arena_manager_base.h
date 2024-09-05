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
#ifndef CYBER_MESSAGE_ARENA_MANAGER_BASE_H_
#define CYBER_MESSAGE_ARENA_MANAGER_BASE_H_

#include <cstdint>
#include <memory>

namespace apollo {
namespace cyber {
namespace message {

class ArenaMessageWrapper;

class ArenaManagerBase {
 public:
  ArenaManagerBase() {}
  virtual ~ArenaManagerBase() {}

  virtual uint64_t GetBaseAddress(const ArenaMessageWrapper* wrapper) {
    return 0;
  }

  virtual void* SetMessage(ArenaMessageWrapper* wrapper,
                           const void* message) = 0;
  virtual void* GetMessage(ArenaMessageWrapper* wrapper) = 0;

  std::shared_ptr<message::ArenaMessageWrapper> CreateMessageWrapper() {
    return std::make_shared<message::ArenaMessageWrapper>(this);
  }

  template <typename MessageT>
  MessageT* SetMessage(ArenaMessageWrapper* wrapper, const MessageT& message) {
    void* msg = SetMessage(wrapper, reinterpret_cast<const void*>(&message));
    return reinterpret_cast<MessageT*>(msg);
  }

  template <typename MessageT>
  MessageT* GetMessage(ArenaMessageWrapper* wrapper) {
    void* msg = GetMessage(wrapper);
    return reinterpret_cast<MessageT*>(msg);
  }
};

}  // namespace message
}  // namespace cyber
}  // namespace apollo

#endif
