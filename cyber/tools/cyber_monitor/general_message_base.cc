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

#include "cyber/tools/cyber_monitor/general_message.h"
#include "cyber/tools/cyber_monitor/screen.h"

namespace {
constexpr int INT_FLOAT_PRECISION = 6;
constexpr int DOULBE_PRECISION = 9;

int CalculateStringLines(const std::string& str, int screen_width) {
  int line_width = 0;
  int line_count = 0;
  for (std::size_t i = 0; i < str.size(); ++i) {
    if (str[i] == '\n' || str[i] == '\r') {
      ++line_count;
      line_width = 0;
    } else {
      ++line_width;
      if (line_width == screen_width) {
        ++line_count;
        line_width = 0;
      }
    }
  }

  if (line_width) {
    ++line_count;
  }

  return line_count;
}

}  // namespace

int GeneralMessageBase::LineCount(const google::protobuf::Message& msg,
                                  int screen_width) {
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
    ret += LineCountOfField(msg, screen_width, fields[i], reflection);
  }

  return ret;
}

int GeneralMessageBase::LineCountOfField(
    const google::protobuf::Message& msg, int screen_width,
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
          ret += CalculateStringLines(value, screen_width);
          break;
        }

        case google::protobuf::FieldDescriptor::CPPTYPE_MESSAGE: {
          const google::protobuf::Message& child_msg =
              reflection->GetRepeatedMessage(msg, field, i);
          ret += LineCount(child_msg, screen_width);
          break;
        }

        default:
          ret += 1;
      }
    }
  } else {
    ret = 1;
    if (!field->is_repeated()) {
      switch (field->cpp_type()) {
        case google::protobuf::FieldDescriptor::CPPTYPE_STRING: {
          std::string scratch;
          const std::string& value =
              reflection->GetStringReference(msg, field, &scratch);
          ret += CalculateStringLines(value, screen_width);
          break;
        }

        case google::protobuf::FieldDescriptor::CPPTYPE_MESSAGE: {
          const google::protobuf::Message& child_msg =
              reflection->GetMessage(msg, field);
          ret += LineCount(child_msg, screen_width);
          break;
        }

        default: {
        }
      }
    }
  }
  return ret;
}

void GeneralMessageBase::PrintMessage(GeneralMessageBase* baseMsg,
                                      const google::protobuf::Message& msg,
                                      int* jump_lines, const Screen* s,
                                      int* line_no, int indent) {
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
    if (*line_no > s->Height()) {
      break;
    }
    const google::protobuf::FieldDescriptor* field = fields[i];
    if (field->is_repeated()) {
      if (*jump_lines) {
        --(*jump_lines);
      } else {
        std::ostringstream out_str;
        const std::string& fieldName = field->name();
        out_str << fieldName << ": ";
        out_str << "+[" << reflection->FieldSize(msg, field) << " items]";
        GeneralMessage* item =
            new GeneralMessage(baseMsg, &msg, reflection, field);
        if (item) {
          baseMsg->InsertRepeatedMessage(*line_no, item);
        }
        s->AddStr(indent, (*line_no)++, out_str.str().c_str());
      }
    } else {
      PrintField(baseMsg, msg, jump_lines, s, line_no, indent, reflection,
                 field, -1);
    }
  }

  const google::protobuf::UnknownFieldSet& unknown_fields =
      reflection->GetUnknownFields(msg);
  if (!unknown_fields.empty()) {
    Screen::ColorPair c = s->Color();
    s->ClearCurrentColor();
    s->SetCurrentColor(Screen::RED_BLACK);
    s->AddStr(indent, (*line_no)++, "Have Unknown Fields");
    s->ClearCurrentColor();
    s->SetCurrentColor(c);
  }
}

void GeneralMessageBase::PrintField(
    GeneralMessageBase* baseMsg, const google::protobuf::Message& msg,
    int* jump_lines, const Screen* s, int* line_no, int indent,
    const google::protobuf::Reflection* ref,
    const google::protobuf::FieldDescriptor* field, int index) {
  std::ostringstream out_str;
  std::ios_base::fmtflags old_flags;

  switch (field->cpp_type()) {
#define OUTPUT_FIELD(CPPTYPE, METHOD, PRECISION)                    \
  case google::protobuf::FieldDescriptor::CPPTYPE_##CPPTYPE:        \
    if (*jump_lines) {                                              \
      --(*jump_lines);                                              \
    } else {                                                        \
      const std::string& fieldName = field->name();                 \
      out_str << fieldName << ": ";                                 \
      if (field->is_repeated()) {                                   \
        out_str << "[" << index << "] ";                            \
      }                                                             \
      old_flags = out_str.flags();                                  \
      out_str << std::fixed << std::setprecision(PRECISION)         \
              << (field->is_repeated()                              \
                      ? ref->GetRepeated##METHOD(msg, field, index) \
                      : ref->Get##METHOD(msg, field));              \
      out_str.flags(old_flags);                                     \
      s->AddStr(indent, (*line_no)++, out_str.str().c_str());       \
    }                                                               \
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
        int line_width = 0;
        std::size_t i = 0;

        for (; i < str.size() && *jump_lines > 0; ++i) {
          if (str[i] == '\n' || str[i] == '\r') {
            --(*jump_lines);
            line_width = 0;
          } else {
            ++line_width;
            if (line_width == s->Width()) {
              --(*jump_lines);
              line_width = 0;
            }
          }
        }

        if (*jump_lines == 0) {
          line_width = 0;
          unsigned line_count = 1;

          const std::string& fieldName = field->name();
          out_str << fieldName << ": ";
          if (field->is_repeated()) {
            out_str << "[" << index << "] ";
          }

          for (; i < str.size(); ++i) {
            char ch = str[i];
            if (str[i] == '\n' || str[i] == '\r') {
              ++line_count;
              line_width = 0;
              ch = '\n';
            } else {
              ++line_width;
              if (line_width == s->Width()) {
                ++line_count;
                line_width = 0;
              }
            }
            out_str << ch;
          }

          s->AddStr(indent, *line_no, out_str.str().c_str());
          (*line_no) += line_count;
        }
      }

      break;
    }

    case google::protobuf::FieldDescriptor::CPPTYPE_ENUM: {
      if (*jump_lines) {
        --(*jump_lines);
      } else {
        const std::string& fieldName = field->name();
        out_str << fieldName << ": ";
        if (field->is_repeated()) {
          out_str << "[" << index << "] ";
        }
        int enum_value = field->is_repeated()
                             ? ref->GetRepeatedEnumValue(msg, field, index)
                             : ref->GetEnumValue(msg, field);
        const google::protobuf::EnumValueDescriptor* enum_desc =
            field->enum_type()->FindValueByNumber(enum_value);
        if (enum_desc != nullptr) {
          out_str << enum_desc->name();
        } else {
          out_str << enum_value;
        }
        s->AddStr(indent, (*line_no)++, out_str.str().c_str());
      }
      break;
    }

    case google::protobuf::FieldDescriptor::CPPTYPE_MESSAGE:
      if (!*jump_lines) {
        const std::string& fieldName = field->name();
        out_str << fieldName;
        if (!field->is_map()) {
          out_str << ": ";
          if (field->is_repeated()) {
            out_str << "[" << index << "] ";
          }
        }
        s->AddStr(indent, (*line_no)++, out_str.str().c_str());
      } else {
        --(*jump_lines);
      }
      GeneralMessageBase::PrintMessage(
          baseMsg,
          field->is_repeated() ? ref->GetRepeatedMessage(msg, field, index)
                               : ref->GetMessage(msg, field),
          jump_lines, s, line_no, indent + 2);
      break;
  }
}

RenderableMessage* GeneralMessageBase::Child(int line_no) const {
  if (line_no < 0) {
    return nullptr;
  }
  auto iter = children_map_.find(line_no);
  if (iter == children_map_.cend()) {
    return nullptr;
  }

  return iter->second;
}
