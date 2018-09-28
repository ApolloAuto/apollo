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

#include "general_message.h"
#include "general_channel_message.h"
#include "screen.h"

#include <sstream>

RenderableMessage* GeneralMessage::Child(int lineNo) const {
  return GeneralMessageBase::Child(lineNo);
}

void GeneralMessage::PrintRepeatedField(const Screen* s, unsigned& lineNo,
                                        int indent, int index, int jumpLines) {
  std::ostringstream outStr;
  std::ios_base::fmtflags old_flags;
  const std::string& fieldName = field_->name();
  outStr << fieldName << ": ";

  if (field_->is_repeated()) {
    outStr << "[" << index << "] ";
  }

  switch (field_->cpp_type()) {
#define OUTPUT_FIELD(CPPTYPE, METHOD, PRECISION)                             \
  case google::protobuf::FieldDescriptor::CPPTYPE_##CPPTYPE:                 \
    old_flags = outStr.flags();                                              \
    outStr << std::fixed << std::setprecision(PRECISION)                     \
           << field_->is_repeated()                                          \
        ? reflection_ptr_->GetRepeated##METHOD(*message_ptr_, field_, index) \
        : reflection_ptr_->Get##METHOD(*message_ptr_, field_);               \
    outStr.flags(old_flags);                                                 \
    break

    OUTPUT_FIELD(INT32, Int32, 6);
    OUTPUT_FIELD(INT64, Int64, 6);
    OUTPUT_FIELD(UINT32, UInt32, 6);
    OUTPUT_FIELD(UINT64, UInt64, 6);
    OUTPUT_FIELD(FLOAT, Float, 6);
    OUTPUT_FIELD(DOUBLE, Double, 9);
    OUTPUT_FIELD(BOOL, Bool, 6);
#undef OUTPUT_FIELD

    case google::protobuf::FieldDescriptor::CPPTYPE_STRING: {
      std::string scratch;
      const std::string& value =
          field_->is_repeated()
              ? reflection_ptr_->GetRepeatedStringReference(
                    *message_ptr_, field_, index, &scratch)
              : reflection_ptr_->GetStringReference(*message_ptr_, field_,
                                                    &scratch);
      outStr << value.substr(jumpLines);
      break;
    }

    case google::protobuf::FieldDescriptor::CPPTYPE_ENUM: {
      int enum_value =
          field_->is_repeated()
              ? reflection_ptr_->GetRepeatedEnumValue(*message_ptr_, field_,
                                                      index)
              : reflection_ptr_->GetEnumValue(*message_ptr_, field_);
      const google::protobuf::EnumValueDescriptor* enum_desc =
          field_->enum_type()->FindValueByNumber(enum_value);
      if (enum_desc != nullptr) {
        outStr << enum_desc->name();
      } else {
        outStr << enum_value;
      }
      break;
    }

    case google::protobuf::FieldDescriptor::CPPTYPE_MESSAGE:
      s->AddStr(indent, lineNo++, outStr.str().c_str());
      GeneralMessageBase::PrintMessage(
          this, field_->is_repeated()
                    ? reflection_ptr_->GetRepeatedMessage(*message_ptr_, field_,
                                                          index)
                    : reflection_ptr_->GetMessage(*message_ptr_, field_),
          s, lineNo, indent + 2, jumpLines + 1);
      outStr.str("");
      break;
  }

  s->AddStr(indent, lineNo++, outStr.str().c_str());
}

GeneralMessage::GeneralMessage(GeneralMessageBase* parent,
                               const google::protobuf::Message* msg,
                               const google::protobuf::Reflection* reflection,
                               const google::protobuf::FieldDescriptor* field)
    : GeneralMessageBase(parent),
      itemIndex_(0),
      field_(field),
      message_ptr_(msg),
      reflection_ptr_(reflection) {}

void GeneralMessage::Render(const Screen* s, int key) {
  s->SetCurrentColor(Screen::WHITE_BLACK);

  {
    unsigned lineNo = 0;

    RenderableMessage* p = this;
    while (p->parent()->parent()->parent()) {
      p = p->parent();
    }

    GeneralChannelMessage* parentPtr =
        static_cast<GeneralChannelMessage*>(p->parent());
    s->AddStr(0, lineNo++, "ChannelName: ");
    s->AddStr(parentPtr->GetChannelName().c_str());

    s->AddStr(0, lineNo++, "MessageType: ");
    s->AddStr(parentPtr->message_type().c_str());

    std::ostringstream outStr;
    outStr << std::fixed << std::setprecision(2) << parentPtr->frame_ratio();
    s->AddStr(0, lineNo++, "FrameRatio: ");
    s->AddStr(outStr.str().c_str());

    clear();

    if (message_ptr_ && reflection_ptr_) {
      int size = 0;
      if (field_->is_repeated()) {
        size = reflection_ptr_->FieldSize(*message_ptr_, field_);
      } else {
        if (reflection_ptr_->HasField(*message_ptr_, field_) ||
            field_->containing_type()->options().map_entry()) {
          size = 1;
        }
      }

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

      int lcount =
          lineCountOfField(*message_ptr_, s->Width(), field_, reflection_ptr_);
      int pageItemCount = s->Height() - lineNo;
      pages_ = lcount / pageItemCount + 1;
      SplitPages(key);

      PrintRepeatedField(s, lineNo, 0, itemIndex_, page_index_ * pageItemCount);
    }
  }

  s->ClearCurrentColor(Screen::WHITE_BLACK);
}
