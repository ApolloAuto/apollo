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

#ifndef CYBER_TRANSPORT_TRANSMITTER_INTRA_TRANSMITTER_H_
#define CYBER_TRANSPORT_TRANSMITTER_INTRA_TRANSMITTER_H_

#include <memory>
#include <string>

#include "cyber/common/log.h"
#include "cyber/transport/dispatcher/intra_dispatcher.h"
#include "cyber/transport/transmitter/transmitter.h"

namespace apollo {
namespace cyber {
namespace transport {

template <typename M>
class IntraTransmitter : public Transmitter<M> {
 public:
  using MessagePtr = std::shared_ptr<M>;

  explicit IntraTransmitter(const RoleAttributes& attr);
  virtual ~IntraTransmitter();

  void Enable() override;
  void Disable() override;

  void Enable(const RoleAttributes& opposite_attr) override;
  void Disable(const RoleAttributes& opposite_attr) override;

  bool Transmit(const MessagePtr& msg, const MessageInfo& msg_info) override;

  bool AcquireMessage(std::shared_ptr<M>& msg);

 private:
  uint64_t channel_id_;
  IntraDispatcherPtr dispatcher_;
};

template <typename M>
bool IntraTransmitter<M>::AcquireMessage(std::shared_ptr<M>& msg) {
  return false;
}

template <typename M>
IntraTransmitter<M>::IntraTransmitter(const RoleAttributes& attr)
    : Transmitter<M>(attr),
      channel_id_(attr.channel_id()),
      dispatcher_(nullptr) {}

template <typename M>
IntraTransmitter<M>::~IntraTransmitter() {
  Disable();
}

template <typename M>
void IntraTransmitter<M>::Enable(const RoleAttributes& opposite_attr) {
  (void)opposite_attr;
  this->Enable();
}

template <typename M>
void IntraTransmitter<M>::Disable(const RoleAttributes& opposite_attr) {
  (void)opposite_attr;
  this->Disable();
}

template <typename M>
void IntraTransmitter<M>::Enable() {
  if (!this->enabled_) {
    dispatcher_ = IntraDispatcher::Instance();
    this->enabled_ = true;
  }
}

template <typename M>
void IntraTransmitter<M>::Disable() {
  if (this->enabled_) {
    dispatcher_ = nullptr;
    this->enabled_ = false;
  }
}

template <typename M>
bool IntraTransmitter<M>::Transmit(const MessagePtr& msg,
                                   const MessageInfo& msg_info) {
  if (!this->enabled_) {
    ADEBUG << "not enable.";
    return false;
  }

  dispatcher_->OnMessage(channel_id_, msg, msg_info);
  return true;
}

}  // namespace transport
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_TRANSPORT_TRANSMITTER_INTRA_TRANSMITTER_H_
