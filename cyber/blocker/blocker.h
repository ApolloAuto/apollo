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

#ifndef CYBER_BLOCKER_BLOCKER_H_
#define CYBER_BLOCKER_BLOCKER_H_

#include <cstddef>
#include <functional>
#include <list>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace apollo {
namespace cyber {
namespace blocker {

class BlockerBase {
 public:
  virtual ~BlockerBase() = default;

  virtual void Reset() = 0;
  virtual void ClearObserved() = 0;
  virtual void ClearPublished() = 0;
  virtual void Observe() = 0;
  virtual bool IsObservedEmpty() const = 0;
  virtual bool IsPublishedEmpty() const = 0;
  virtual bool Unsubscribe(const std::string& callback_id) = 0;

  virtual size_t capacity() const = 0;
  virtual void set_capacity(size_t capacity) = 0;
  virtual const std::string& channel_name() const = 0;
};

struct BlockerAttr {
  BlockerAttr() : capacity(10), channel_name("") {}
  explicit BlockerAttr(const std::string& channel)
      : capacity(10), channel_name(channel) {}
  BlockerAttr(size_t cap, const std::string& channel)
      : capacity(cap), channel_name(channel) {}
  BlockerAttr(const BlockerAttr& attr)
      : capacity(attr.capacity), channel_name(attr.channel_name) {}

  size_t capacity;
  std::string channel_name;
};

template <typename T>
class Blocker : public BlockerBase {
  friend class BlockerManager;

 public:
  using MessageType = T;
  using MessagePtr = std::shared_ptr<T>;
  using MessageQueue = std::list<MessagePtr>;
  using Callback = std::function<void(const MessagePtr&)>;
  using CallbackMap = std::unordered_map<std::string, Callback>;
  using Iterator = typename std::list<std::shared_ptr<T>>::const_iterator;

  explicit Blocker(const BlockerAttr& attr);
  virtual ~Blocker();

  void Publish(const MessageType& msg);
  void Publish(const MessagePtr& msg);

  void ClearObserved() override;
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
  void Reset() override;
  void Enqueue(const MessagePtr& msg);
  void Notify(const MessagePtr& msg);

  BlockerAttr attr_;
  MessageQueue observed_msg_queue_;
  MessageQueue published_msg_queue_;
  mutable std::mutex msg_mutex_;

  CallbackMap published_callbacks_;
  mutable std::mutex cb_mutex_;

  MessageType dummy_msg_;
};

template <typename T>
Blocker<T>::Blocker(const BlockerAttr& attr) : attr_(attr), dummy_msg_() {}

template <typename T>
Blocker<T>::~Blocker() {
  published_msg_queue_.clear();
  observed_msg_queue_.clear();
  published_callbacks_.clear();
}

template <typename T>
void Blocker<T>::Publish(const MessageType& msg) {
  Publish(std::make_shared<MessageType>(msg));
}

template <typename T>
void Blocker<T>::Publish(const MessagePtr& msg) {
  Enqueue(msg);
  Notify(msg);
}

template <typename T>
void Blocker<T>::Reset() {
  {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    observed_msg_queue_.clear();
    published_msg_queue_.clear();
  }
  {
    std::lock_guard<std::mutex> lock(cb_mutex_);
    published_callbacks_.clear();
  }
}

template <typename T>
void Blocker<T>::ClearObserved() {
  std::lock_guard<std::mutex> lock(msg_mutex_);
  observed_msg_queue_.clear();
}

template <typename T>
void Blocker<T>::ClearPublished() {
  std::lock_guard<std::mutex> lock(msg_mutex_);
  published_msg_queue_.clear();
}

template <typename T>
void Blocker<T>::Observe() {
  std::lock_guard<std::mutex> lock(msg_mutex_);
  observed_msg_queue_ = published_msg_queue_;
}

template <typename T>
bool Blocker<T>::IsObservedEmpty() const {
  std::lock_guard<std::mutex> lock(msg_mutex_);
  return observed_msg_queue_.empty();
}

template <typename T>
bool Blocker<T>::IsPublishedEmpty() const {
  std::lock_guard<std::mutex> lock(msg_mutex_);
  return published_msg_queue_.empty();
}

template <typename T>
bool Blocker<T>::Subscribe(const std::string& callback_id,
                           const Callback& callback) {
  std::lock_guard<std::mutex> lock(cb_mutex_);
  if (published_callbacks_.find(callback_id) != published_callbacks_.end()) {
    return false;
  }
  published_callbacks_[callback_id] = callback;
  return true;
}

template <typename T>
bool Blocker<T>::Unsubscribe(const std::string& callback_id) {
  std::lock_guard<std::mutex> lock(cb_mutex_);
  return published_callbacks_.erase(callback_id) != 0;
}

template <typename T>
auto Blocker<T>::GetLatestObserved() const -> const MessageType& {
  std::lock_guard<std::mutex> lock(msg_mutex_);
  if (observed_msg_queue_.empty()) {
    return dummy_msg_;
  }
  return *observed_msg_queue_.front();
}

template <typename T>
auto Blocker<T>::GetLatestObservedPtr() const -> const MessagePtr {
  std::lock_guard<std::mutex> lock(msg_mutex_);
  if (observed_msg_queue_.empty()) {
    return nullptr;
  }
  return observed_msg_queue_.front();
}

template <typename T>
auto Blocker<T>::GetOldestObservedPtr() const -> const MessagePtr {
  std::lock_guard<std::mutex> lock(msg_mutex_);
  if (observed_msg_queue_.empty()) {
    return nullptr;
  }
  return observed_msg_queue_.back();
}

template <typename T>
auto Blocker<T>::GetLatestPublishedPtr() const -> const MessagePtr {
  std::lock_guard<std::mutex> lock(msg_mutex_);
  if (published_msg_queue_.empty()) {
    return nullptr;
  }
  return published_msg_queue_.front();
}

template <typename T>
auto Blocker<T>::ObservedBegin() const -> Iterator {
  return observed_msg_queue_.begin();
}

template <typename T>
auto Blocker<T>::ObservedEnd() const -> Iterator {
  return observed_msg_queue_.end();
}

template <typename T>
size_t Blocker<T>::capacity() const {
  return attr_.capacity;
}

template <typename T>
void Blocker<T>::set_capacity(size_t capacity) {
  std::lock_guard<std::mutex> lock(msg_mutex_);
  attr_.capacity = capacity;
  while (published_msg_queue_.size() > capacity) {
    published_msg_queue_.pop_back();
  }
}

template <typename T>
const std::string& Blocker<T>::channel_name() const {
  return attr_.channel_name;
}

template <typename T>
void Blocker<T>::Enqueue(const MessagePtr& msg) {
  if (attr_.capacity == 0) {
    return;
  }
  std::lock_guard<std::mutex> lock(msg_mutex_);
  published_msg_queue_.push_front(msg);
  while (published_msg_queue_.size() > attr_.capacity) {
    published_msg_queue_.pop_back();
  }
}

template <typename T>
void Blocker<T>::Notify(const MessagePtr& msg) {
  std::lock_guard<std::mutex> lock(cb_mutex_);
  for (const auto& item : published_callbacks_) {
    item.second(msg);
  }
}

}  // namespace blocker
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_BLOCKER_BLOCKER_H_
