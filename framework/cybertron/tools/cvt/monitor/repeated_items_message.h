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
#include "general_message_base.h"
// #include "renderable_message.h"

class GeneralMessage;
class Screen;

class RepeatedItemsMessage : public GeneralMessageBase {
 public:
  enum { Type = GeneralMessageBase::Type + 2 };
  explicit RepeatedItemsMessage(GeneralMessage* parent, const google::protobuf::Message* msg, const google::protobuf::FieldDescriptor* field);
  explicit RepeatedItemsMessage(RepeatedItemsMessage* parent, const google::protobuf::Message* msg, 
                                /* const google::protobuf::Reflection* reflection, */
                                const google::protobuf::FieldDescriptor* field);

  ~RepeatedItemsMessage() { }

  int type(void) const { return Type; }

  void Render(const Screen* s, int key) override;
  bool isRR(void) const { return drawType_ == RR; }
  bool isGR(void) const { return drawType_ == GR; }

  RenderableMessage* Child(int lineNo) const override;

 private:
  RepeatedItemsMessage(const RepeatedItemsMessage&) = delete;
  RepeatedItemsMessage& operator=(const RepeatedItemsMessage&) = delete;

  enum DrawType{ GR, RR, RG, GG, };

  void PrintRepeatedField(const Screen* s, unsigned& lineNo, int indent,
                                 const google::protobuf::Message& message,
                                 const google::protobuf::Reflection* reflection,
                                 const google::protobuf::FieldDescriptor* field,
                                 int index);

  void drawGR(const Screen* s, int key);
  void drawRR(const Screen* s, int key);

  int itemIndex_;

  const google::protobuf::FieldDescriptor* field_;
  RepeatedItemsMessage* repeatedItemListParent_;
  const google::protobuf::Message* message_ptr_;
  const google::protobuf::Reflection* reflection_ptr_;
  DrawType drawType_;
};

#endif  // TOOLS_CVT_MONITOR_REPEATED_ITEMS_MESSAGE_H_
