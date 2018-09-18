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

#ifndef TOOLS_CVT_MONITOR_GENERAL_MESSAGE_H_
#define TOOLS_CVT_MONITOR_GENERAL_MESSAGE_H_

#include <cybertron/message/raw_message.h>
#include "cybertron_channel_message.h"

class RepeatedItemsMessage;

BegDefineChannelMsgSubClass(GeneralMessage,
                            apollo::cybertron::message::RawMessage);
SubClassDeconstructor(GeneralMessage) {
  if (raw_msg_class_) {
    delete raw_msg_class_;
    raw_msg_class_ = nullptr;
  }

  for(auto& iter : children_map_){
      delete iter.second;
  }
}

RenderableMessage* Child(int lineNo) const override;

SubClassConstructor(GeneralMessage, apollo::cybertron::message::RawMessage),
    current_state_(State::ShowDebugString), page_index_(0),
    raw_msg_class_(nullptr), children_map_() {}

void RenderDebugString(const Screen* s, int key, unsigned lineNo);
void RenderInfo(const Screen* s, int key, unsigned lineNo);
void SplitPages(int key);

static void PrintFieldValue(const google::protobuf::Message& msg, 
  const google::protobuf::Reflection* reflection, 
  const Screen* s, unsigned& lineNo, int indent, 
  const google::protobuf::FieldDescriptor* field);

static void PrintMessage(const google::protobuf::Message& msg, const Screen* s,
                  unsigned& lineNo, int indent, int jumpLines = 0);

enum class State { ShowDebugString, ShowInfo, ShowRepeatedItems } current_state_;

int pages_;
int page_index_;
google::protobuf::Message* raw_msg_class_;

std::map<const int /* lineNo */, RepeatedItemsMessage*> children_map_;

friend class RepeatedItemsMessage;

EndDefineChannelMsgSubClass(GeneralMessage);

#endif  // TOOLS_CVT_MONITOR_GENERAL_MESSAGE_H_
