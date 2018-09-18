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

void RepeatedItemsMessage::PrintFieldValue(
    const Screen* s, unsigned& lineNo, int indent,
    const google::protobuf::Message& message,
    const google::protobuf::Reflection* reflection,
    const google::protobuf::FieldDescriptor* field, int index) {
  std::ostringstream outStr;
  const std::string& fieldName = field->name();
  outStr << fieldName << ": ";

  switch (field->cpp_type()) {
#define OUTPUT_FIELD(CPPTYPE, METHOD)                                 \
  case google::protobuf::FieldDescriptor::CPPTYPE_##CPPTYPE:          \
    outStr << reflection->GetRepeated##METHOD(message, field, index); \
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
      const std::string& value = reflection->GetRepeatedStringReference(
          message, field, index, &scratch);
      outStr << value;
      break;
    }

    case google::protobuf::FieldDescriptor::CPPTYPE_ENUM: {
      int enum_value = reflection->GetRepeatedEnumValue(message, field, index);
      outStr << enum_value;
      break;
    }

    case google::protobuf::FieldDescriptor::CPPTYPE_MESSAGE:
      outStr << "[" << index << "]";
      s->AddStr(indent, lineNo++, outStr.str().c_str());
      GeneralMessage::PrintMessage(
          reflection->GetRepeatedMessage(message, field, index), s, lineNo, indent + 2);
      outStr.str("");
      break;
  }

  s->AddStr(indent, lineNo++, outStr.str().c_str());
}

RepeatedItemsMessage::RepeatedItemsMessage(GeneralMessage* parent,
                                           int fieldIndex)
    : RenderableMessage(parent),
      fieldIndex_(fieldIndex),
      itemIndex_(0),
      channel_message_(parent->channel_message_) {}

void RepeatedItemsMessage::Render(const Screen* s, int key) {
  unsigned lineNo = 0;

  GeneralMessage* parentPtr = static_cast<GeneralMessage*>(parent());
  s->SetCurrentColor(Screen::WHITE_BLACK);
  s->AddStr(0, lineNo++, "ChannelName: ");
  s->AddStr(parentPtr->GetChannelName().c_str());

  s->AddStr(0, lineNo++, "MessageType: ");
  s->AddStr(parentPtr->message_type().c_str());

  std::ostringstream outStr;
  outStr << std::fixed << std::setprecision(2) << parentPtr->frame_ratio();
  s->AddStr(0, lineNo++, "FrameRatio: ");
  s->AddStr(outStr.str().c_str());

  if (parentPtr->raw_msg_class_->ParseFromString(channel_message_->message)) {
    const google::protobuf::Descriptor* descriptor =
        parentPtr->raw_msg_class_->GetDescriptor();
    const google::protobuf::Reflection* reflection =
        parentPtr->raw_msg_class_->GetReflection();

    std::vector<const google::protobuf::FieldDescriptor*> fields;
    reflection->ListFields(*(parentPtr->raw_msg_class_), &fields);

    const google::protobuf::FieldDescriptor* field = fields[fieldIndex_];

    int size = reflection->FieldSize(*(parentPtr->raw_msg_class_), field);

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

    RepeatedItemsMessage::PrintFieldValue(s, lineNo, 0,
                                          *(parentPtr->raw_msg_class_),
                                          reflection, field, itemIndex_);
  }
}
