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

#ifndef CYBERTRON_DISPATCHER_MESSAGE_H_
#define CYBERTRON_DISPATCHER_MESSAGE_H_

#include <assert.h>
#include <stddef.h>
#include <functional>
#include <list>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace apollo {
namespace cybertron {
namespace dispatcher {

class MessageBase {
 public:
  virtual ~MessageBase() = default;

  virtual void ClearPublished() = 0;
  virtual void Observe() = 0;
  virtual bool IsObservedEmpty() const = 0;
  virtual bool IsPublishedEmpty() const = 0;
  virtual bool Unsubscribe(const std::string& callback_id) = 0;

  virtual size_t capacity() const = 0;
  virtual void set_capacity(size_t capacity) = 0;
  virtual const std::string& channel_name() const = 0;
};

struct MessageAttr {
  MessageAttr() : capacity(10), channel_name("") {}
  explicit MessageAttr(const std::string& channel)
      : capacity(10), channel_name(channel) {}
  MessageAttr(size_t cap, const std::string& channel)
      : capacity(cap), channel_name(channel) {}
  MessageAttr(const MessageAttr& attr)
      : capacity(attr.capacity), channel_name(attr.channel_name) {}

  size_t capacity;
  std::string channel_name;
};

template <typename T>
class Message : public MessageBase {
 public:
  using MessageType = T;
  using MessagePtr = std::shared_ptr<T>;
  using MessageQueue = std::list<MessagePtr>;
  using Callback = std::function<void(const MessagePtr&)>;
  using CallbackMap = std::unordered_map<std::string, Callback>;
  using Iterator = typename std::list<std::shared_ptr<T>>::const_iterator;

  explicit Message(const MessageAttr& attr);
  virtual ~Message();

  void Publish(const MessageType& msg);
  void Publish(const MessagePtr& msg);

  void ClearPublished() override;
  void Observe() override;
  bool IsObservedEmpty() const override;
  bool IsPublishedEmpty() const override;

  bool Subscribe(const std::string& callback_id, const Callback& callback);
  bool Unsubscribe(const std::string& callback_id) override;

  const MessageType& GetLatestObserved() const;
  const MessagePtr GetLatestObservedPtr() const;
  const MessagePtr GetOldestObservedPtr() const;
  const MessagePtr GetLatestPublishedPtr() const;

  Iterator ObservedBegin() const;
  Iterator ObservedEnd() const;

  size_t capacity() const override;
  void set_capacity(size_t capacity) override;
  const std::string& channel_name() const override;

 private:
  void Enqueue(const MessagePtr& msg);
  void Notify(const MessagePtr& msg);

  bool is_full_;
  MessageAttr attr_;
  MessageQueue observed_msg_queue_;
  MessageQueue published_msg_queue_;
  mutable std::mutex msg_mutex_;

  CallbackMap published_callbacks_;
  mutable std::mutex cb_mutex_;
};

template <typename T>
Message<T>::Message(const MessageAttr& attr) : is_full_(false), attr_(attr) {}

template <typename T>
Message<T>::~Message() {
  published_msg_queue_.clear();
  observed_msg_queue_.clear();
  published_callbacks_.clear();
}

template <typename T>
void Message<T>::Publish(const MessageType& msg) {
  Publish(std::make_shared<MessageType>(msg));
}

template <typename T>
void Message<T>::Publish(const MessagePtr& msg) {
  Enqueue(msg);
  Notify(msg);
}

template <typename T>
void Message<T>::ClearPublished() {
  std::lock_guard<std::mutex> lock(msg_mutex_);
  published_msg_queue_.clear();
}

template <typename T>
void Message<T>::Observe() {
  std::lock_guard<std::mutex> lock(msg_mutex_);
  observed_msg_queue_ = published_msg_queue_;
}

template <typename T>
bool Message<T>::IsObservedEmpty() const {
  std::lock_guard<std::mutex> lock(msg_mutex_);
  return observed_msg_queue_.empty();
}

template <typename T>
bool Message<T>::IsPublishedEmpty() const {
  std::lock_guard<std::mutex> lock(msg_mutex_);
  return published_msg_queue_.empty();
}

template <typename T>
bool Message<T>::Subscribe(const std::string& callback_id,
                           const Callback& callback) {
  std::lock_guard<std::mutex> lock(cb_mutex_);
  if (published_callbacks_.find(callback_id) != published_callbacks_.end()) {
    return false;
  }
  published_callbacks_[callback_id] = callback;
  return true;
}

template <typename T>
bool Message<T>::Unsubscribe(const std::string& callback_id) {
  std::lock_guard<std::mutex> lock(cb_mutex_);
  return published_callbacks_.erase(callback_id) != 0;
}

template <typename T>
auto Message<T>::GetLatestObserved() const -> const MessageType& {
  std::lock_guard<std::mutex> lock(msg_mutex_);
  assert(!observed_msg_queue_.empty());
  return *observed_msg_queue_.back();
}

template <typename T>
auto Message<T>::GetLatestObservedPtr() const -> const MessagePtr {
  std::lock_guard<std::mutex> lock(msg_mutex_);
  assert(!observed_msg_queue_.empty());
  return observed_msg_queue_.back();
}

template <typename T>
auto Message<T>::GetOldestObservedPtr() const -> const MessagePtr {
  std::lock_guard<std::mutex> lock(msg_mutex_);
  assert(!observed_msg_queue_.empty());
  return observed_msg_queue_.front();
}

template <typename T>
auto Message<T>::GetLatestPublishedPtr() const -> const MessagePtr {
  std::lock_guard<std::mutex> lock(msg_mutex_);
  assert(!published_msg_queue_.empty());
  return published_msg_queue_.back();
}

template <typename T>
auto Message<T>::ObservedBegin() const -> Iterator {
  return observed_msg_queue_.begin();
}

template <typename T>
auto Message<T>::ObservedEnd() const -> Iterator {
  return observed_msg_queue_.end();
}

template <typename T>
size_t Message<T>::capacity() const {
  return attr_.capacity;
}

template <typename T>
void Message<T>::set_capacity(size_t capacity) {
  std::lock_guard<std::mutex> lock(msg_mutex_);
  if (capacity > attr_.capacity) {
    is_full_ = false;
  }
  attr_.capacity = capacity;
  while (published_msg_queue_.size() > capacity) {
    published_msg_queue_.pop_front();
  }
}

template <typename T>
const std::string& Message<T>::channel_name() const {
  return attr_.channel_name;
}

template <typename T>
void Message<T>::Enqueue(const MessagePtr& msg) {
  if (attr_.capacity == 0) {
    return;
  }
  std::lock_guard<std::mutex> lock(msg_mutex_);
  if (is_full_) {
    published_msg_queue_.pop_front();
  }

  published_msg_queue_.push_back(msg);

  if (!is_full_) {
    if (published_msg_queue_.size() >= attr_.capacity) {
      is_full_ = true;
    }
  }
}

template <typename T>
void Message<T>::Notify(const MessagePtr& msg) {
  std::lock_guard<std::mutex> lock(cb_mutex_);
  for (const auto& item : published_callbacks_) {
    item.second(msg);
  }
}

}  // namespace dispatcher
}  // namespace cybertron
}  // namespace apollo

#endif  // CYBERTRON_DISPATCHER_MESSAGE_H_
