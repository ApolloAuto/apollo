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

#include "repeated_items_message.h"
#include "general_message.h"
#include "screen.h"

#include <sstream>

RenderableMessage* RepeatedItemsMessage::Child(int lineNo) const {
  return GeneralMessageBase::Child(lineNo);
}

void RepeatedItemsMessage::PrintRepeatedField(
    const Screen* s, unsigned& lineNo, int indent,
    const google::protobuf::Message& message,
    const google::protobuf::Reflection* reflection,
    const google::protobuf::FieldDescriptor* field, int index) {
  std::ostringstream outStr;
  const std::string& fieldName = field->name();
  outStr << fieldName << ": ";

  switch (field->cpp_type()) {
#define OUTPUT_FIELD(CPPTYPE, METHOD)                            \
  case google::protobuf::FieldDescriptor::CPPTYPE_##CPPTYPE:     \
    outStr << field->is_repeated()                               \
        ? reflection->GetRepeated##METHOD(message, field, index) \
        : reflection->Get##METHOD(message, field);               \
    break

    OUTPUT_FIELD(INT32, Int32);
    OUTPUT_FIELD(INT64, Int64);
    OUTPUT_FIELD(UINT32, UInt32);
    OUTPUT_FIELD(UINT64, UInt64);
    OUTPUT_FIELD(FLOAT, Float);
    OUTPUT_FIELD(DOUBLE, Double);
    OUTPUT_FIELD(BOOL, Bool);
#undef OUTPUT_FIELD

    case google::protobuf::FieldDescriptor::CPPTYPE_STRING: {
      std::string scratch;
      const std::string& value = field->is_repeated() ? reflection->GetRepeatedStringReference(
          message, field, index, &scratch) : reflection->GetStringReference(message, field, &scratch);
      outStr << value;
      break;
    }

    case google::protobuf::FieldDescriptor::CPPTYPE_ENUM: {
      int enum_value = field->is_repeated() ? reflection->GetRepeatedEnumValue(message, field, index) :
                  reflection->GetEnumValue(message, field);
      outStr << enum_value;
      break;
    }

    case google::protobuf::FieldDescriptor::CPPTYPE_MESSAGE:
      outStr << "[" << index << "]";
      s->AddStr(indent, lineNo++, outStr.str().c_str());
          GeneralMessageBase::PrintMessage(
              this, field->is_repeated() ? reflection->GetRepeatedMessage(message, field, index) : reflection->GetMessage(message, field), s,
              lineNo, indent + 2);
          outStr.str("");
      break;
  }

  s->AddStr(indent, lineNo++, outStr.str().c_str());
}

RepeatedItemsMessage::RepeatedItemsMessage(
    GeneralMessage* parent, const google::protobuf::Message* msg, const google::protobuf::FieldDescriptor* field)
    : GeneralMessageBase(parent),
      field_(field),repeatedItemListParent_(nullptr),
      itemIndex_(0),
      message_ptr_(msg),
      drawType_(GR) {}

RepeatedItemsMessage::RepeatedItemsMessage(
    RepeatedItemsMessage* parent, const google::protobuf::Message* msg, /* const google::protobuf::Reflection* reflection, */
    const google::protobuf::FieldDescriptor* field)
    : GeneralMessageBase(parent),
      field_(field), repeatedItemListParent_(parent),
      itemIndex_(0),
      message_ptr_(msg), reflection_ptr_(/* reflection */ nullptr),
      drawType_(RR) {}

void RepeatedItemsMessage::Render(const Screen* s, int key) {
  s->SetCurrentColor(Screen::WHITE_BLACK);

  if (isGR()) {
    drawGR(s, key);
  } else {
    drawRR(s, key);
  }

  s->ClearCurrentColor(Screen::WHITE_BLACK);
}

// const google::protobuf::FieldDescriptor* RepeatedItemsMessage::getFiled(
//     const google::protobuf::Message& msg, int index) {
//   const google::protobuf::Reflection* reflection = msg.GetReflection();

//   std::vector<const google::protobuf::FieldDescriptor*> fields;
//   reflection->ListFields(msg, &fields);

//   return fields[index];
// }

// const google::protobuf::Message*  RepeatedItemsMessage::upperMsg(void){
//   if(upper_msg_) return upper_msg_;
//   else return static_cast<GeneralMessage*>(parent())->raw_msg_class_;
// }

void RepeatedItemsMessage::drawGR(const Screen* s, int key) {
  unsigned lineNo = 0;

  GeneralMessage* parentPtr = static_cast<GeneralMessage*>(parent());
  s->AddStr(0, lineNo++, "ChannelName: ");
  s->AddStr(parentPtr->GetChannelName().c_str());

  s->AddStr(0, lineNo++, "MessageType: ");
  s->AddStr(parentPtr->message_type().c_str());

  std::ostringstream outStr;
  outStr << std::fixed << std::setprecision(2) << parentPtr->frame_ratio();
  s->AddStr(0, lineNo++, "FrameRatio: ");
  s->AddStr(outStr.str().c_str());

  if (message_ptr_) {

    const google::protobuf::Reflection* reflection =
        message_ptr_->GetReflection();

    int size = 0;
    if(field_->is_repeated())
      size = reflection->FieldSize(*message_ptr_, field_);

    switch (key) {
      case 'n':
      case 'N':
        ++itemIndex_;
        if (itemIndex_ >= size) itemIndex_ = 0;
        break;

      case 'm':
      case 'M':
        --itemIndex_;
        if (itemIndex_ < 0) itemIndex_ = size - 1;
        break;

      default:;
    }

    PrintRepeatedField(s, lineNo, 0, *message_ptr_, reflection,
                       field_, itemIndex_);
  }
}
void RepeatedItemsMessage::drawRR(const Screen* s, int key) {
  unsigned lineNo = 0;
  s->AddStr(0, lineNo, "in drawRR");

  RepeatedItemsMessage* p = this;
  while(p->repeatedItemListParent_){
    p = p->repeatedItemListParent_;
  }

  GeneralMessage* parentPtr = static_cast<GeneralMessage*>(p->parent());
  s->AddStr(0, lineNo++, "ChannelName: ");
  s->AddStr(parentPtr->GetChannelName().c_str());

  s->AddStr(0, lineNo++, "MessageType: ");
  s->AddStr(parentPtr->message_type().c_str());

  std::ostringstream outStr;
  outStr << std::fixed << std::setprecision(2) << parentPtr->frame_ratio();
  s->AddStr(0, lineNo++, "FrameRatio: ");
  s->AddStr(outStr.str().c_str());

  if (message_ptr_) {
    const google::protobuf::Reflection* reflection =
        message_ptr_->GetReflection();

    s->AddStr(0, lineNo++, field_->type_name());

    int size = 0;
    if(field_->is_repeated())
      size = reflection->FieldSize(*message_ptr_, field_);

    switch (key) {
      case 'n':
      case 'N':
        ++itemIndex_;
        if (itemIndex_ >= size) itemIndex_ = 0;
        break;

      case 'm':
      case 'M':
        --itemIndex_;
        if (itemIndex_ < 0) itemIndex_ = size - 1;
        break;

      default:;
    }

    PrintRepeatedField(s, lineNo, 0, *message_ptr_, reflection,
                       field_, itemIndex_);
  }
}
