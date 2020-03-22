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

#include "cyber/tools/cyber_monitor/general_message_base.h"

#include <iomanip>
#include <string>
#include <vector>

#include "cyber/tools/cyber_monitor/general_channel_message.h"
#include "cyber/tools/cyber_monitor/general_message.h"
#include "cyber/tools/cyber_monitor/screen.h"

namespace {
constexpr int INT_FLOAT_PRECISION = 6;
constexpr int DOULBE_PRECISION = 9;

int calculateStringLines(const std::string& str, int screenWidth) {
  int lineWidth = 0;
  int lineCount = 0;
  for (std::size_t i = 0; i < str.size(); ++i) {
    if (str[i] == '\n' || str[i] == '\r') {
      ++lineCount;
      lineWidth = 0;
    } else {
      ++lineWidth;
      if (lineWidth == screenWidth) {
        ++lineCount;
        lineWidth = 0;
      }
    }
  }

  if (lineWidth) {
    ++lineCount;
  }

  return lineCount;
}

}  // namespace

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

  auto fsize = fields.size();
  int ret = 0;
  for (decltype(fsize) i = 0; i < fsize; ++i) {
    ret += lineCountOfField(msg, screenWidth, fields[i], reflection);
  }  // end for

  return ret;
}

int GeneralMessageBase::lineCountOfField(
    const google::protobuf::Message& msg, int screenWidth,
    const google::protobuf::FieldDescriptor* field,
    const google::protobuf::Reflection* reflection, bool is_folded) {
  int ret = 0;
  if (!is_folded && field->is_repeated()) {
    int size = reflection->FieldSize(msg, field);
    for (int i = 0; i < size; ++i) {
      switch (field->cpp_type()) {
        case google::protobuf::FieldDescriptor::CPPTYPE_STRING: {
          std::string scratch;
          const std::string& value =
              reflection->GetRepeatedStringReference(msg, field, i, &scratch);
          ret += calculateStringLines(value, screenWidth);
          break;
        }

        case google::protobuf::FieldDescriptor::CPPTYPE_MESSAGE: {
          const google::protobuf::Message& childMsg =
              reflection->GetRepeatedMessage(msg, field, i);
          ret += lineCount(childMsg, screenWidth);
          break;
        }

        default:
          ret += 1;
      }  // end switch
    }
  } else {
    ret = 1;
    if (!field->is_repeated()) {
      switch (field->cpp_type()) {
        case google::protobuf::FieldDescriptor::CPPTYPE_STRING: {
          std::string scratch;
          const std::string& value =
              reflection->GetStringReference(msg, field, &scratch);
          ret += calculateStringLines(value, screenWidth);
          break;
        }

        case google::protobuf::FieldDescriptor::CPPTYPE_MESSAGE: {
          const google::protobuf::Message& childMsg =
              reflection->GetMessage(msg, field);
          ret += lineCount(childMsg, screenWidth);
          break;
        }

        default: {}
      }  // end switch
    }
  }
  return ret;
}

void GeneralMessageBase::PrintMessage(GeneralMessageBase* baseMsg,
                                      const google::protobuf::Message& msg,
                                      int& jumpLines, const Screen* s,
                                      unsigned& lineNo, int indent) {
  const google::protobuf::Reflection* reflection = msg.GetReflection();
  const google::protobuf::Descriptor* descriptor = msg.GetDescriptor();
  std::vector<const google::protobuf::FieldDescriptor*> fields;
  if (descriptor->options().map_entry()) {
    fields.push_back(descriptor->field(0));
    fields.push_back(descriptor->field(1));
  } else {
    reflection->ListFields(msg, &fields);
  }
  for (std::size_t i = 0; i < fields.size(); ++i) {
    if (lineNo > static_cast<unsigned>(s->Height())) {
      break;
    }
    const google::protobuf::FieldDescriptor* field = fields[i];
    if (field->is_repeated()) {
      if (jumpLines) {
        --jumpLines;
      } else {
        std::ostringstream outStr;
        const std::string& fieldName = field->name();
        outStr << fieldName << ": ";
        outStr << "+[" << reflection->FieldSize(msg, field) << " items]";
        GeneralMessage* item =
            new GeneralMessage(baseMsg, &msg, reflection, field);
        if (item) {
          baseMsg->insertRepeatedMessage(lineNo, item);
        }
        s->AddStr(indent, lineNo++, outStr.str().c_str());
      }
    } else {
      PrintField(baseMsg, msg, jumpLines, s, lineNo, indent, reflection, field,
                 -1);
    }  // end else
  }    // end for

  const google::protobuf::UnknownFieldSet& unknown_fields =
      reflection->GetUnknownFields(msg);
  if (!unknown_fields.empty()) {
    Screen::ColorPair c = s->Color();
    s->ClearCurrentColor();
    s->SetCurrentColor(Screen::RED_BLACK);
    s->AddStr(indent, lineNo++, "Have Unknown Fields");
    s->ClearCurrentColor();
    s->SetCurrentColor(c);
  }
}

void GeneralMessageBase::PrintField(
    GeneralMessageBase* baseMsg, const google::protobuf::Message& msg,
    int& jumpLines, const Screen* s, unsigned& lineNo, int indent,
    const google::protobuf::Reflection* ref,
    const google::protobuf::FieldDescriptor* field, int index) {
  std::ostringstream outStr;
  std::ios_base::fmtflags old_flags;

  switch (field->cpp_type()) {
#define OUTPUT_FIELD(CPPTYPE, METHOD, PRECISION)                   \
  case google::protobuf::FieldDescriptor::CPPTYPE_##CPPTYPE:       \
    if (jumpLines) {                                               \
      --jumpLines;                                                 \
    } else {                                                       \
      const std::string& fieldName = field->name();                \
      outStr << fieldName << ": ";                                 \
      if (field->is_repeated()) {                                  \
        outStr << "[" << index << "] ";                            \
      }                                                            \
      old_flags = outStr.flags();                                  \
      outStr << std::fixed << std::setprecision(PRECISION)         \
             << (field->is_repeated()                              \
                     ? ref->GetRepeated##METHOD(msg, field, index) \
                     : ref->Get##METHOD(msg, field));              \
      outStr.flags(old_flags);                                     \
      s->AddStr(indent, lineNo++, outStr.str().c_str());           \
    }                                                              \
    break

    OUTPUT_FIELD(INT32, Int32, INT_FLOAT_PRECISION);
    OUTPUT_FIELD(INT64, Int64, INT_FLOAT_PRECISION);
    OUTPUT_FIELD(UINT32, UInt32, INT_FLOAT_PRECISION);
    OUTPUT_FIELD(UINT64, UInt64, INT_FLOAT_PRECISION);
    OUTPUT_FIELD(FLOAT, Float, INT_FLOAT_PRECISION);
    OUTPUT_FIELD(DOUBLE, Double, DOULBE_PRECISION);
    OUTPUT_FIELD(BOOL, Bool, INT_FLOAT_PRECISION);
#undef OUTPUT_FIELD

    case google::protobuf::FieldDescriptor::CPPTYPE_STRING: {
      std::string scratch;
      const std::string& str =
          field->is_repeated()
              ? ref->GetRepeatedStringReference(msg, field, index, &scratch)
              : ref->GetStringReference(msg, field, &scratch);
      {
        int lineWidth = 0;
        std::size_t i = 0;

        for (; i < str.size() && jumpLines > 0; ++i) {
          if (str[i] == '\n' || str[i] == '\r') {
            --jumpLines;
            lineWidth = 0;
          } else {
            ++lineWidth;
            if (lineWidth == s->Width()) {
              --jumpLines;
              lineWidth = 0;
            }
          }
        }

        if (jumpLines == 0) {
          lineWidth = 0;
          unsigned lineCount = 1;

          const std::string& fieldName = field->name();
          outStr << fieldName << ": ";
          if (field->is_repeated()) {
            outStr << "[" << index << "] ";
          }

          for (; i < str.size(); ++i) {
            char ch = str[i];
            if (str[i] == '\n' || str[i] == '\r') {
              ++lineCount;
              lineWidth = 0;
              ch = '\n';
            } else {
              ++lineWidth;
              if (lineWidth == s->Width()) {
                ++lineCount;
                lineWidth = 0;
              }
            }
            outStr << ch;
          }

          s->AddStr(indent, lineNo, outStr.str().c_str());
          lineNo += lineCount;
        }
      }

      break;
    }

    case google::protobuf::FieldDescriptor::CPPTYPE_ENUM: {
      if (jumpLines) {
        --jumpLines;
      } else {
        const std::string& fieldName = field->name();
        outStr << fieldName << ": ";
        if (field->is_repeated()) {
          outStr << "[" << index << "] ";
        }
        int enum_value = field->is_repeated()
                             ? ref->GetRepeatedEnumValue(msg, field, index)
                             : ref->GetEnumValue(msg, field);
        const google::protobuf::EnumValueDescriptor* enum_desc =
            field->enum_type()->FindValueByNumber(enum_value);
        if (enum_desc != nullptr) {
          outStr << enum_desc->name();
        } else {
          outStr << enum_value;
        }
        s->AddStr(indent, lineNo++, outStr.str().c_str());
      }
      break;
    }

    case google::protobuf::FieldDescriptor::CPPTYPE_MESSAGE:
      if (!jumpLines) {
        const std::string& fieldName = field->name();
        outStr << fieldName;
        if (!field->is_map()) {
          outStr << ": ";
          if (field->is_repeated()) {
            outStr << "[" << index << "] ";
          }
        }
        s->AddStr(indent, lineNo++, outStr.str().c_str());
      } else {
        --jumpLines;
      }
      GeneralMessageBase::PrintMessage(
          baseMsg,
          field->is_repeated() ? ref->GetRepeatedMessage(msg, field, index)
                               : ref->GetMessage(msg, field),
          jumpLines, s, lineNo, indent + 2);
      break;
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
