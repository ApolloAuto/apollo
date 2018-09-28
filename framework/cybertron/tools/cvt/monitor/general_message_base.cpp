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

#include "general_message_base.h"
#include "general_channel_message.h"
#include "general_message.h"
#include "screen.h"

#include <iomanip>

int GeneralMessageBase::lineCount(const google::protobuf::Message& msg,
                                  int screenWidth) {
  const google::protobuf::Reflection* reflection = msg.GetReflection();
  const google::protobuf::Descriptor* descriptor = msg.GetDescriptor();
  std::vector<const google::protobuf::FieldDescriptor*> fields;
  if (descriptor->options().map_entry()) {
    fields.push_back(descriptor->field(0));
    fields.push_back(descriptor->field(1));
  } else {
    reflection->ListFields(msg, &fields);
  }

  int fsize = fields.size();
  int ret = 0;
  for (int i = 0; i < fsize; ++i) {
    ret += lineCountOfField(msg, screenWidth, fields[i], reflection);
  }  // end for

  return ret;
}

int GeneralMessageBase::lineCountOfField(
    const google::protobuf::Message& msg, int screenWidth,
    const google::protobuf::FieldDescriptor* field,
    const google::protobuf::Reflection* reflection) {
  int ret = 1;
  if (!field->is_repeated()) {
    switch (field->cpp_type()) {
      case google::protobuf::FieldDescriptor::CPPTYPE_STRING: {
        std::string scratch;
        const std::string& value =
            reflection->GetStringReference(msg, field, &scratch);
        ret += value.size() / screenWidth;
        break;
      }

      case google::protobuf::FieldDescriptor::CPPTYPE_MESSAGE:
        const google::protobuf::Message& childMsg =
            reflection->GetMessage(msg, field);
        ret += lineCount(childMsg, screenWidth) + 1;
        break;

    }  // end switch
  }

  return ret;
}

void GeneralMessageBase::PrintMessage(GeneralMessageBase* baseMsg,
                                      const google::protobuf::Message& msg,
                                      const Screen* s, unsigned& lineNo,
                                      int indent, int jumpLines) {
  const google::protobuf::Reflection* reflection = msg.GetReflection();
  const google::protobuf::Descriptor* descriptor = msg.GetDescriptor();
  std::vector<const google::protobuf::FieldDescriptor*> fields;
  if (descriptor->options().map_entry()) {
    fields.push_back(descriptor->field(0));
    fields.push_back(descriptor->field(1));
  } else {
    reflection->ListFields(msg, &fields);
  }

  int i = 0;
  // jump lines
  for (; i < fields.size() && jumpLines > 1; ++i) {
    const google::protobuf::FieldDescriptor* field = fields[i];
    --jumpLines;

    if (!field->is_repeated()) {
      switch (field->cpp_type()) {
        case google::protobuf::FieldDescriptor::CPPTYPE_STRING: {
          std::string scratch;
          const std::string& value =
              reflection->GetStringReference(msg, field, &scratch);
          jumpLines -= value.size() / s->Width();
          break;
        }

        case google::protobuf::FieldDescriptor::CPPTYPE_MESSAGE:
          jumpLines -=
              lineCount(reflection->GetMessage(msg, field), s->Width());
          break;
      }  // end switch
    }
  }

  std::ostringstream outStr;
  for (; i < fields.size(); ++i) {
    const google::protobuf::FieldDescriptor* field = fields[i];
    const std::string& fieldName = field->name();
    outStr << fieldName << ": ";
    if (field->is_repeated()) {
      outStr << "+[" << reflection->FieldSize(msg, field) << " items]";

      GeneralMessage* item =
          new GeneralMessage(baseMsg, &msg, reflection, field);

      if (item) {
        baseMsg->insertRepeatedMessage(lineNo, item);
      }

    } else {
      switch (field->cpp_type()) {
#define OUTPUT_FIELD(CPPTYPE, METHOD)                                       \
  case google::protobuf::FieldDescriptor::CPPTYPE_##CPPTYPE:                \
    outStr << std::setprecision(64) << reflection->Get##METHOD(msg, field); \
    break

        OUTPUT_FIELD(INT32, Int32);
        OUTPUT_FIELD(INT64, Int64);
        OUTPUT_FIELD(UINT32, UInt32);
        OUTPUT_FIELD(UINT64, UInt64);
        OUTPUT_FIELD(FLOAT, Float);
        OUTPUT_FIELD(DOUBLE, Double);
        OUTPUT_FIELD(BOOL, Bool);
#undef OUTPUT_FIELD

        case google::protobuf::FieldDescriptor::CPPTYPE_ENUM: {
          int enum_value = reflection->GetEnumValue(msg, field);

          const google::protobuf::EnumValueDescriptor* enum_desc =
              field->enum_type()->FindValueByNumber(enum_value);
          if (enum_desc != nullptr) {
            outStr << enum_desc->name() << "  " << enum_value;
          } else {
            outStr << enum_value << "  " << enum_value;
          }
          break;
        }

        case google::protobuf::FieldDescriptor::CPPTYPE_STRING: {
          std::string scratch;
          const std::string& value =
              reflection->GetStringReference(msg, field, &scratch);
          outStr << value;
          break;
        }

        case google::protobuf::FieldDescriptor::CPPTYPE_MESSAGE:
          s->AddStr(indent, lineNo++, outStr.str().c_str());
          PrintMessage(baseMsg, reflection->GetMessage(msg, field), s, lineNo,
                       indent + 2);
          outStr.str("");
          break;
      }  // end switch
    }    // end else

    s->AddStr(indent, lineNo++, outStr.str().c_str());
    outStr.str("");
  }  // end for

  const google::protobuf::UnknownFieldSet& unknown_fields =
      reflection->GetUnknownFields(msg);
  if (!unknown_fields.empty()) {
    Screen::ColorPair c = s->Color();
    s->ClearCurrentColor(c);
    s->SetCurrentColor(Screen::RED_BLACK);
    s->AddStr(indent, lineNo++, "Have Unknown Fields");
    s->ClearCurrentColor(Screen::RED_BLACK);
    s->SetCurrentColor(c);
  }
}

RenderableMessage* GeneralMessageBase::Child(int lineNo) const {
  if (lineNo < 0) {
    return nullptr;
  }
  auto iter = children_map_.find(lineNo);
  if (iter == children_map_.cend()) {
    return nullptr;
  }

  return iter->second;
}
