/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#ifndef CYBERTRON_TRANSPORT_LOWER_REACH_INTRA_LOWER_REACH_H_
#define CYBERTRON_TRANSPORT_LOWER_REACH_INTRA_LOWER_REACH_H_

#include "cybertron/common/log.h"
#include "cybertron/transport/dispatcher/intra_dispatcher.h"
#include "cybertron/transport/lower_reach/lower_reach.h"

namespace apollo {
namespace cybertron {
namespace transport {

template <typename MessageT>
class IntraLowerReach : public LowerReach<MessageT> {
 public:
  IntraLowerReach(
      const RoleAttributes& attr,
      const typename LowerReach<MessageT>::MessageListener& msg_listener);
  virtual ~IntraLowerReach();

  void Enable() override;
  void Disable() override;

  void Enable(const RoleAttributes& opposite_attr) override;
  void Disable(const RoleAttributes& opposite_attr) override;

 private:
  IntraDispatcherPtr dispatcher_;
};

template <typename MessageT>
IntraLowerReach<MessageT>::IntraLowerReach(
    const RoleAttributes& attr,
    const typename LowerReach<MessageT>::MessageListener& msg_listener)
    : LowerReach<MessageT>(attr, msg_listener) {
  dispatcher_ = IntraDispatcher::Instance();
}

template <typename MessageT>
IntraLowerReach<MessageT>::~IntraLowerReach() {
  Disable();
}

template <typename MessageT>
void IntraLowerReach<MessageT>::Enable() {
  if (this->enabled_) {
    return;
  }
  dispatcher_->AddListener<MessageT>(
      this->attr_, std::bind(&IntraLowerReach<MessageT>::OnNewMessage, this,
                             std::placeholders::_1, std::placeholders::_2));
  this->enabled_ = true;
}

template <typename MessageT>
void IntraLowerReach<MessageT>::Disable() {
  if (!this->enabled_) {
    return;
  }
  dispatcher_->RemoveListener<MessageT>(this->attr_);
  this->enabled_ = false;
}

template <typename MessageT>
void IntraLowerReach<MessageT>::Enable(const RoleAttributes& opposite_attr) {
  dispatcher_->AddListener<MessageT>(
      this->attr_, opposite_attr,
      std::bind(&IntraLowerReach<MessageT>::OnNewMessage, this,
                std::placeholders::_1, std::placeholders::_2));
}

template <typename MessageT>
void IntraLowerReach<MessageT>::Disable(const RoleAttributes& opposite_attr) {
  dispatcher_->RemoveListener<MessageT>(this->attr_, opposite_attr);
}

}  // namespace transport
}  // namespace cybertron
}  // namespace apollo

#endif  // CYBERTRON_TRANSPORT_LOWER_REACH_INTRA_LOWER_REACH_H_
