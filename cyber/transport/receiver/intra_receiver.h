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

#ifndef CYBER_TRANSPORT_RECEIVER_INTRA_RECEIVER_H_
#define CYBER_TRANSPORT_RECEIVER_INTRA_RECEIVER_H_

#include "cyber/common/log.h"
#include "cyber/transport/dispatcher/intra_dispatcher.h"
#include "cyber/transport/receiver/receiver.h"

namespace apollo {
namespace cyber {
namespace transport {

template <typename M>
class IntraReceiver : public Receiver<M> {
 public:
  IntraReceiver(const RoleAttributes& attr,
                const typename Receiver<M>::MessageListener& msg_listener);
  virtual ~IntraReceiver();

  void Enable() override;
  void Disable() override;

  void Enable(const RoleAttributes& opposite_attr) override;
  void Disable(const RoleAttributes& opposite_attr) override;

 private:
  IntraDispatcherPtr dispatcher_;
};

template <typename M>
IntraReceiver<M>::IntraReceiver(
    const RoleAttributes& attr,
    const typename Receiver<M>::MessageListener& msg_listener)
    : Receiver<M>(attr, msg_listener) {
  dispatcher_ = IntraDispatcher::Instance();
}

template <typename M>
IntraReceiver<M>::~IntraReceiver() {
  Disable();
}

template <typename M>
void IntraReceiver<M>::Enable() {
  if (this->enabled_) {
    return;
  }

  dispatcher_->AddListener<M>(
      this->attr_, std::bind(&IntraReceiver<M>::OnNewMessage, this,
                             std::placeholders::_1, std::placeholders::_2));
  this->enabled_ = true;
}

template <typename M>
void IntraReceiver<M>::Disable() {
  if (!this->enabled_) {
    return;
  }

  dispatcher_->RemoveListener<M>(this->attr_);
  this->enabled_ = false;
}

template <typename M>
void IntraReceiver<M>::Enable(const RoleAttributes& opposite_attr) {
  dispatcher_->AddListener<M>(
      this->attr_, opposite_attr,
      std::bind(&IntraReceiver<M>::OnNewMessage, this, std::placeholders::_1,
                std::placeholders::_2));
}

template <typename M>
void IntraReceiver<M>::Disable(const RoleAttributes& opposite_attr) {
  dispatcher_->RemoveListener<M>(this->attr_, opposite_attr);
}

}  // namespace transport
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_TRANSPORT_RECEIVER_INTRA_RECEIVER_H_
