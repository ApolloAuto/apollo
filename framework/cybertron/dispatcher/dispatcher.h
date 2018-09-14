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

#ifndef CYBERTRON_DISPATCHER_DISPATCHER_H_
#define CYBERTRON_DISPATCHER_DISPATCHER_H_

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include "cybertron/dispatcher/message.h"

namespace apollo {
namespace cybertron {
namespace dispatcher {

class Dispatcher {
 public:
  using MessageMap =
      std::unordered_map<std::string, std::shared_ptr<MessageBase>>;

  virtual ~Dispatcher();

  static const std::shared_ptr<Dispatcher>& Instance() {
    static auto instance = std::shared_ptr<Dispatcher>(new Dispatcher());
    return instance;
  }

  template <typename T>
  bool Publish(const std::string& channel_name,
               const typename Message<T>::MessagePtr& msg);

  template <typename T>
  bool Publish(const std::string& channel_name,
               const typename Message<T>::MessageType& msg);

  template <typename T>
  bool Subscribe(const std::string& channel_name, size_t capacity,
                 const std::string& callback_id,
                 const typename Message<T>::Callback& callback);

  template <typename T>
  bool Unsubscribe(const std::string& channel_name,
                   const std::string& callback_id);

  template <typename T>
  std::shared_ptr<Message<T>> GetMessage(const std::string& channel_name);

  template <typename T>
  std::shared_ptr<Message<T>> GetOrCreateMessage(const MessageAttr& attr);

  void Observe();

 private:
  Dispatcher();
  Dispatcher(const Dispatcher&) = delete;
  Dispatcher& operator=(const Dispatcher&) = delete;

  MessageMap messages_;
  std::mutex msg_mutex_;
};

template <typename T>
bool Dispatcher::Publish(const std::string& channel_name,
                         const typename Message<T>::MessagePtr& msg) {
  auto message = GetOrCreateMessage<T>(MessageAttr(channel_name));
  if (message == nullptr) {
    return false;
  }
  message->Publish(msg);
  return true;
}

template <typename T>
bool Dispatcher::Publish(const std::string& channel_name,
                         const typename Message<T>::MessageType& msg) {
  auto message = GetOrCreateMessage<T>(MessageAttr(channel_name));
  if (message == nullptr) {
    return false;
  }
  message->Publish(msg);
  return true;
}

template <typename T>
bool Dispatcher::Subscribe(const std::string& channel_name, size_t capacity,
                           const std::string& callback_id,
                           const typename Message<T>::Callback& callback) {
  auto message = GetOrCreateMessage<T>(MessageAttr(capacity, channel_name));
  if (message == nullptr) {
    return false;
  }
  return message->Subscribe(callback_id, callback);
}

template <typename T>
bool Dispatcher::Unsubscribe(const std::string& channel_name,
                             const std::string& callback_id) {
  auto message = GetMessage<T>(channel_name);
  if (message == nullptr) {
    return false;
  }
  return message->Unsubscribe(callback_id);
}

template <typename T>
std::shared_ptr<Message<T>> Dispatcher::GetMessage(
    const std::string& channel_name) {
  std::shared_ptr<Message<T>> message = nullptr;
  {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    auto search = messages_.find(channel_name);
    if (search != messages_.end()) {
      message = std::dynamic_pointer_cast<Message<T>>(search->second);
    }
  }
  return message;
}

template <typename T>
std::shared_ptr<Message<T>> Dispatcher::GetOrCreateMessage(
    const MessageAttr& attr) {
  std::shared_ptr<Message<T>> message = nullptr;
  {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    auto search = messages_.find(attr.channel_name);
    if (search != messages_.end()) {
      message = std::dynamic_pointer_cast<Message<T>>(search->second);
    } else {
      message = std::make_shared<Message<T>>(attr);
      messages_[attr.channel_name] = message;
    }
  }
  return message;
}

}  // namespace dispatcher
}  // namespace cybertron
}  // namespace apollo

#endif  // CYBERTRON_DISPATCHER_DISPATCHER_H_
