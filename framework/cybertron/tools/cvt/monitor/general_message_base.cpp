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
#include "screen.h"
#include "repeated_items_message.h"
#include "general_message.h"

int GeneralMessageBase::lineCount(const google::protobuf::Message& msg,
                                  int screenWidth) {
  // const google::protobuf::Descriptor* descriptor = msg.GetDescriptor();
  const google::protobuf::Reflection* reflection = msg.GetReflection();

  std::vector<const google::protobuf::FieldDescriptor*> fields;
  reflection->ListFields(msg, &fields);

  int fsize = fields.size();
  int ret = 0;
  for (int i = 0; i < fsize; ++i, ++ret) {
    const google::protobuf::FieldDescriptor* field = fields[i];
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
  }  // end for

  return ret;
}

void GeneralMessageBase::PrintMessage(GeneralMessageBase* baseMsg,
                                      const google::protobuf::Message& msg,
                                      const Screen* s, unsigned& lineNo,
                                      int indent, int jumpLines) {
  const google::protobuf::Reflection* reflection = msg.GetReflection();

  std::vector<const google::protobuf::FieldDescriptor*> fields;
  reflection->ListFields(msg, &fields);

  int i = 0;
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

        RepeatedItemsMessage* item = nullptr;
        if(baseMsg->type() == RepeatedItemsMessage::Type){
          item = new RepeatedItemsMessage(static_cast<RepeatedItemsMessage*>(baseMsg), &msg, field);
        } else {
          item = new RepeatedItemsMessage(static_cast<GeneralMessage*>(baseMsg), &msg, field);
        }

        if(item) {
          baseMsg->insertRepeatedMessage(lineNo, item);
        }

    } else {
      switch (field->cpp_type()) {
#define OUTPUT_FIELD(CPPTYPE, METHOD)                        \
  case google::protobuf::FieldDescriptor::CPPTYPE_##CPPTYPE: \
    outStr << reflection->Get##METHOD(msg, field);           \
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
          outStr << enum_value;
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
}

RenderableMessage* GeneralMessageBase::Child(int lineNo) const{
  if (lineNo < 0) {
    return nullptr;
  }
  auto iter = children_map_.find(lineNo);
  if (iter == children_map_.cend()) {
    return nullptr;
  }

  return iter->second;
}
