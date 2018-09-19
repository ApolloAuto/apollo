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

#ifndef CYBERTRON_BLOCKER_INTRA_READER_H_
#define CYBERTRON_BLOCKER_INTRA_READER_H_

#include <functional>
#include <list>
#include <memory>

#include "cybertron/blocker/blocker.h"
#include "cybertron/blocker/blocker_manager.h"
#include "cybertron/node/reader.h"

namespace apollo {
namespace cybertron {
namespace blocker {

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
  std::shared_ptr<MessageT> GetLatestObserved() const override;
  std::shared_ptr<MessageT> GetOldestObserved() const override;
  Iterator Begin() const override;
  Iterator End() const override;

 private:
  void OnMessage(const MessagePtr& msg_ptr);

  Callback msg_callback_;
  std::shared_ptr<Blocker<MessageT>> blocker_;
};

template <typename MessageT>
IntraReader<MessageT>::IntraReader(const proto::RoleAttributes& attr,
                                   const Callback& callback)
    : Reader<MessageT>(attr), msg_callback_(callback), blocker_(nullptr) {}

template <typename MessageT>
IntraReader<MessageT>::~IntraReader() {
  Shutdown();
}

template <typename MessageT>
bool IntraReader<MessageT>::Init() {
  if (this->init_.exchange(true)) {
    return true;
  }
  BlockerAttr attr(this->role_attr_.qos_profile().depth(),
                   this->role_attr_.channel_name());
  blocker_ = BlockerManager::Instance()->GetOrCreateBlocker<MessageT>(attr);
  if (blocker_ == nullptr) {
    return false;
  }
  return blocker_->Subscribe(this->role_attr_.node_name(),
                             std::bind(&IntraReader<MessageT>::OnMessage, this,
                                       std::placeholders::_1));
}

template <typename MessageT>
void IntraReader<MessageT>::Shutdown() {
  if (!this->init_.exchange(false)) {
    return;
  }
  blocker_->Unsubscribe(this->role_attr_.node_name());
  blocker_ = nullptr;
}

template <typename MessageT>
void IntraReader<MessageT>::ClearData() {
  if (blocker_ == nullptr) {
    return;
  }
  blocker_->ClearPublished();
}

template <typename MessageT>
void IntraReader<MessageT>::Observe() {
  if (blocker_ == nullptr) {
    return;
  }
  blocker_->Observe();
}

template <typename MessageT>
bool IntraReader<MessageT>::Empty() const {
  if (blocker_ == nullptr) {
    return true;
  }
  return blocker_->IsObservedEmpty();
}

template <typename MessageT>
bool IntraReader<MessageT>::HasReceived() const {
  if (blocker_ == nullptr) {
    return false;
  }
  return !blocker_->IsPublishedEmpty();
}

template <typename MessageT>
void IntraReader<MessageT>::Enqueue(const std::shared_ptr<MessageT>& msg) {
  if (blocker_ == nullptr) {
    return;
  }
  blocker_->Publish(msg);
}

template <typename MessageT>
void IntraReader<MessageT>::SetHistoryDepth(const uint32_t& depth) {
  if (blocker_ == nullptr) {
    return;
  }
  blocker_->set_capacity(depth);
}

template <typename MessageT>
uint32_t IntraReader<MessageT>::GetHistoryDepth() const {
  if (blocker_ == nullptr) {
    return 0;
  }
  return blocker_->capacity();
}

template <typename MessageT>
std::shared_ptr<MessageT> IntraReader<MessageT>::GetLatestObserved()
    const {
  if (blocker_ == nullptr) {
    return nullptr;
  }
  return blocker_->GetLatestObservedPtr();
}

template <typename MessageT>
std::shared_ptr<MessageT> IntraReader<MessageT>::GetOldestObserved()
    const {
  if (blocker_ == nullptr) {
    return nullptr;
  }
  return blocker_->GetOldestObservedPtr();
}

template <typename MessageT>
auto IntraReader<MessageT>::Begin() const -> Iterator {
  assert(blocker_ != nullptr);
  return blocker_->ObservedBegin();
}

template <typename MessageT>
auto IntraReader<MessageT>::End() const -> Iterator {
  assert(blocker_ != nullptr);
  return blocker_->ObservedEnd();
}

template <typename MessageT>
void IntraReader<MessageT>::OnMessage(const MessagePtr& msg_ptr) {
  if (msg_callback_ != nullptr) {
    msg_callback_(msg_ptr);
  }
}

}  // namespace blocker
}  // namespace cybertron
}  // namespace apollo

#endif  // CYBERTRON_BLOCKER_INTRA_READER_H_
