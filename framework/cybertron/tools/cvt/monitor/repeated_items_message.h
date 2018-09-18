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

#ifndef TOOLS_CVT_MONITOR_REPEATED_ITEMS_MESSAGE_H_
#define TOOLS_CVT_MONITOR_REPEATED_ITEMS_MESSAGE_H_

#include <cybertron/cybertron.h>
#include <cybertron/message/raw_message.h>
#include "renderable_message.h"

class GeneralMessage;
class Screen;

class RepeatedItemsMessage : public RenderableMessage {
 public:
  explicit RepeatedItemsMessage(GeneralMessage* parent, int fieldIndex);
  ~RepeatedItemsMessage() {}

  void Render(const Screen* s, int key) override;

 private:
  RepeatedItemsMessage(const RepeatedItemsMessage&) = delete;
  RepeatedItemsMessage& operator=(const RepeatedItemsMessage&) = delete;

  static void PrintFieldValue(const Screen* s, unsigned& lineNo, int indent,
                              const google::protobuf::Message& message,
                              const google::protobuf::Reflection* reflection,
                              const google::protobuf::FieldDescriptor* field, int index);

  int fieldIndex_;
  int itemIndex_;
  std::shared_ptr<apollo::cybertron::message::RawMessage> channel_message_;
};

#endif  // TOOLS_CVT_MONITOR_REPEATED_ITEMS_MESSAGE_H_