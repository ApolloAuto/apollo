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

#ifndef CYBERTRON_DISPATCHER_INTRA_READER_H_
#define CYBERTRON_DISPATCHER_INTRA_READER_H_

#include <functional>
#include <memory>

#include "cybertron/dispatcher/dispatcher.h"
#include "cybertron/node/reader.h"

namespace apollo {
namespace cybertron {
namespace dispatcher {

template <typename MessageT>
class IntraReader : public apollo::cybertron::Reader<MessageT> {
 public:
  using MessagePtr = std::shared_ptr<MessageT>;
  using Callback = std::function<void(const std::shared_ptr<MessageT>&)>;
  using Iterator =
      typename std::list<std::shared_ptr<MessageT>>::const_iterator;

  IntraReader(const proto::RoleAttributes& attr, const Callback& callback);
  virtual ~IntraReader();

  bool Init() override;
  void Shutdown() override;

  void ClearData() override;
  void Observe() override;
  bool Empty() const override;
  bool HasReceived() const override;

  void Enqueue(const std::shared_ptr<MessageT>& msg) override;
  void SetHistoryDepth(const uint32_t& depth) override;
  uint32_t GetHistoryDepth() const override;
  const std::shared_ptr<MessageT>& GetLatestObserved() const override;
  const std::shared_ptr<MessageT>& GetOldestObserved() const override;
  Iterator Begin() const override;
  Iterator End() const override;

 private:
  void OnMessage(const MessagePtr& msg_ptr);

  Callback msg_callback_;
  std::shared_ptr<Message<MessageT>> message_;
};

template <typename MessageT>
IntraReader<MessageT>::IntraReader(const proto::RoleAttributes& attr,
                                   const Callback& callback)
    : Reader<MessageT>(attr), msg_callback_(callback), message_(nullptr) {}

template <typename MessageT>
IntraReader<MessageT>::~IntraReader() {
  Shutdown();
}

template <typename MessageT>
bool IntraReader<MessageT>::Init() {
  if (this->init_.exchange(true)) {
    return true;
  }
  MessageAttr attr(this->role_attr_.qos_profile().depth(),
                   this->role_attr_.channel_name());
  message_ = Dispatcher::Instance()->GetOrCreateMessage<MessageT>(attr);
  if (message_ == nullptr) {
    return false;
  }
  return message_->Subscribe(this->role_attr_.node_name(),
                             std::bind(&IntraReader<MessageT>::OnMessage, this,
                                       std::placeholders::_1));
}

template <typename MessageT>
void IntraReader<MessageT>::Shutdown() {
  if (!this->init_.exchange(false)) {
    return;
  }
  message_->Unsubscribe(this->role_attr_.node_name());
  message_ = nullptr;
}

template <typename MessageT>
void IntraReader<MessageT>::ClearData() {
  if (message_ == nullptr) {
    return;
  }
  message_->ClearPublished();
}

template <typename MessageT>
void IntraReader<MessageT>::Observe() {
  if (message_ == nullptr) {
    return;
  }
  message_->Observe();
}

template <typename MessageT>
bool IntraReader<MessageT>::Empty() const {
  if (message_ == nullptr) {
    return true;
  }
  return message_->IsObservedEmpty();
}

template <typename MessageT>
bool IntraReader<MessageT>::HasReceived() const {
  if (message_ == nullptr) {
    return false;
  }
  return !message_->IsPublishedEmpty();
}

template <typename MessageT>
void IntraReader<MessageT>::Enqueue(const std::shared_ptr<MessageT>& msg) {
  if (message_ == nullptr) {
    return;
  }
  message_->Publish(msg);
}

template <typename MessageT>
void IntraReader<MessageT>::SetHistoryDepth(const uint32_t& depth) {
  if (message_ == nullptr) {
    return;
  }
  message_->set_capacity(depth);
}

template <typename MessageT>
uint32_t IntraReader<MessageT>::GetHistoryDepth() const {
  if (message_ == nullptr) {
    return 0;
  }
  return message_->capacity();
}

template <typename MessageT>
const std::shared_ptr<MessageT>& IntraReader<MessageT>::GetLatestObserved()
    const {
  if (message_ == nullptr) {
    return nullptr;
  }
  return message_->GetLatestObservedPtr();
}

template <typename MessageT>
const std::shared_ptr<MessageT>& IntraReader<MessageT>::GetOldestObserved()
    const {
  if (message_ == nullptr) {
    return nullptr;
  }
  return message_->GetOldestObservedPtr();
}

template <typename MessageT>
auto IntraReader<MessageT>::Begin() const -> Iterator {
  assert(message_ != nullptr);
  return message_->ObservedBegin();
}

template <typename MessageT>
auto IntraReader<MessageT>::End() const -> Iterator {
  assert(message_ != nullptr);
  return message_->ObservedEnd();
}

template <typename MessageT>
void IntraReader<MessageT>::OnMessage(const MessagePtr& msg_ptr) {
  if (msg_callback_ != nullptr) {
    msg_callback_(msg_ptr);
  }
}

}  // namespace dispatcher
}  // namespace cybertron
}  // namespace apollo

#endif  // CYBERTRON_DISPATCHER_INTRA_READER_H_
