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

#include "cyber/message/raw_message.h"
#include "cyber_channel_message.h"
#include "general_message_base.h"

class RepeatedItemsMessage;

class GeneralChannelMessage
    : public CyberChannelMessage<apollo::cyber::message::RawMessage> {
 public:
  ChannelMsgSubFactory(GeneralChannelMessage,
                       apollo::cyber::message::RawMessage);
  void Render(const Screen* s, int key) override;
  ~GeneralChannelMessage() {
    if (raw_msg_class_) {
      delete raw_msg_class_;
      raw_msg_class_ = nullptr;
    }
  }

  RenderableMessage* Child(int lineNo) const override;

 private:
  explicit GeneralChannelMessage(RenderableMessage* parent = nullptr)
      : CyberChannelMessage<apollo::cyber::message::RawMessage>(parent),
        current_state_(State::ShowDebugString),
        raw_msg_class_(nullptr) {}
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
