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

#ifndef TOOLS_CVT_MONITOR_GENERAL_CHANNEL_MESSAGE_H_
#define TOOLS_CVT_MONITOR_GENERAL_CHANNEL_MESSAGE_H_

#include <cybertron/message/raw_message.h>
#include "cybertron_channel_message.h"
#include "general_message_base.h"

class RepeatedItemsMessage;

class GeneralChannelMessage
    : public CybertronChannelMessage<apollo::cybertron::message::RawMessage> {
 public:
  RegisterChannelMsgClass(GeneralChannelMessage,
                          apollo::cybertron::message::RawMessage);
  virtual void Render(const Screen* s, int key) override;
  ~GeneralChannelMessage() {
    if (raw_msg_class_) {
      delete raw_msg_class_;
      raw_msg_class_ = nullptr;
    }
  }

  RenderableMessage* Child(int lineNo) const override;

  explicit GeneralChannelMessage(RenderableMessage* parent = nullptr)
      : CybertronChannelMessage<apollo::cybertron::message::RawMessage>(parent),
        current_state_(State::ShowDebugString),
        // pages_(0),
        // page_index_(0),
        raw_msg_class_(nullptr) {}

 private:
  GeneralChannelMessage(const GeneralChannelMessage&) = delete;
  GeneralChannelMessage& operator=(const GeneralChannelMessage&) = delete;

  void RenderDebugString(const Screen* s, int key, unsigned lineNo);
  void RenderInfo(const Screen* s, int key, unsigned lineNo);
  
  enum class State { ShowDebugString, ShowInfo } current_state_;

  google::protobuf::Message* raw_msg_class_;

  friend class RepeatedItemsMessage;
  friend class GeneralMessageBase;

};  // GeneralChannelMessage

#endif  // TOOLS_CVT_MONITOR_GENERAL_CHANNEL_MESSAGE_H_
