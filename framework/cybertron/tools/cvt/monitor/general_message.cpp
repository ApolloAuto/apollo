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
#include "repeated_items_message.h"
#include "screen.h"

#include <ncurses.h>
#include <iomanip>
#include <sstream>

namespace {
constexpr int ReaderWriterOffset = 4;
constexpr int PageItemCountOffset = 3;
int lineCount(const google::protobuf::Message& msg, int screenWidth,
              std::map<const int, RepeatedItemsMessage*>* childrenMap = nullptr,
              GeneralMessage* gMsg = nullptr) {
  const google::protobuf::Descriptor* descriptor = msg.GetDescriptor();
  const google::protobuf::Reflection* reflection = msg.GetReflection();

  std::vector<const google::protobuf::FieldDescriptor*> fields;
  reflection->ListFields(msg, &fields);

  int fsize = fields.size();
  int ret = 0;
  for (int i = 0; i < fsize; ++i, ++ret) {
    const google::protobuf::FieldDescriptor* field = fields[i];
    if (childrenMap && field->is_repeated()) {
      RepeatedItemsMessage* item = new RepeatedItemsMessage(gMsg, i);
      childrenMap->insert(std::make_pair(ret, item));
    }

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
}  // namespace

void GeneralMessage::PrintMessage(const google::protobuf::Message& msg, const Screen* s,
                  unsigned& lineNo, int indent, int jumpLines) {
  // const google::protobuf::Descriptor* descriptor = msg.GetDescriptor();
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

  for (; i < fields.size(); ++i) {
    GeneralMessage::PrintFieldValue(msg, reflection, s, lineNo, indent,
                                    fields[i]);
  }  // end for
}

void GeneralMessage::PrintFieldValue(
    const google::protobuf::Message& msg,
    const google::protobuf::Reflection* reflection, const Screen* s,
    unsigned& lineNo, int indent,
    const google::protobuf::FieldDescriptor* field) {
      
  const std::string& fieldName = field->name();
  std::ostringstream outStr;

  outStr << fieldName << ": ";
  if (field->is_repeated()) {
    outStr << "+[" << reflection->FieldSize(msg, field) << " items]";
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
        PrintMessage(reflection->GetMessage(msg, field), s, lineNo, indent + 2);
        outStr.str("");
        break;
    }  // end switch
  }    // end else

  s->AddStr(indent, lineNo++, outStr.str().c_str());
}

RenderableMessage* GeneralMessage::Child(int lineNo) const {
  lineNo -= 3;  // 3 is the fixed offset value, 0 for ChannelName, 1 for
                // MessageType, 2 for FrameRatio
  if (lineNo < 0) {
    return nullptr;
  }
  auto iter = children_map_.find(lineNo);
  if (iter == children_map_.cend()) {
    return nullptr;
  }

  return iter->second;
}

void GeneralMessage::Render(const Screen* s, int key) {
  switch (key) {
    case 'b':
    case 'B':
      current_state_ = State::ShowDebugString;
      break;

    case 'i':
    case 'I':
      current_state_ = State::ShowInfo;
      break;

    default:;
  }

  unsigned lineNo = 0;

  s->SetCurrentColor(Screen::WHITE_BLACK);
  s->AddStr(0, lineNo++, "ChannelName: ");
  s->AddStr(channel_reader_->GetChannelName().c_str());

  s->AddStr(0, lineNo++, "MessageType: ");
  s->AddStr(message_type().c_str());
  // ++lineNo;

  switch (current_state_) {
    case State::ShowDebugString:
      RenderDebugString(s, key, lineNo);
      break;
    case State::ShowInfo:
      RenderInfo(s, key, lineNo);
      break;
  }
  s->ClearCurrentColor(Screen::WHITE_BLACK);
}

void GeneralMessage::SplitPages(int key) {
  switch (key) {
    case CTRL('d'):
    case KEY_NPAGE:
      ++page_index_;
      if (page_index_ >= pages_) page_index_ = pages_ - 1;
      break;

    case CTRL('u'):
    case KEY_PPAGE:
      --page_index_;
      if (page_index_ < 1) page_index_ = 0;
      break;
    default:;
  }
}

void GeneralMessage::RenderInfo(const Screen* s, int key, unsigned lineNo) {
  int pageItemCount = s->Height() - PageItemCountOffset;
  pages_ = (readers_.size() + writers_.size() + PageItemCountOffset) /
               pageItemCount +
           1;
  SplitPages(key);

  bool hasReader = true;
  std::vector<std::string>* vec = &readers_;

  auto iter = vec->cbegin();
  int y = page_index_ * pageItemCount;
  if (y < vec->size()) {
    while (y < page_index_ * pageItemCount) {
      ++iter;
      ++y;
    }
  } else {
    y -= vec->size();
    vec = &writers_;
    iter = vec->cbegin();
    while (y) {
      ++iter;
      --y;
    }

    hasReader = false;
  }

  if (hasReader) {
    s->AddStr(0, lineNo++, "Readers:");
    for (; iter != vec->cend(); ++iter) {
      s->AddStr(ReaderWriterOffset, lineNo++, iter->c_str());
    }

    ++lineNo;
    vec = &writers_;
    iter = vec->cbegin();
  }

  s->AddStr(0, lineNo++, "Writers:");
  for (; iter != vec->cend(); ++iter) {
    s->AddStr(ReaderWriterOffset, lineNo++, iter->c_str());
  }
}

void GeneralMessage::RenderDebugString(const Screen* s, int key,
                                       unsigned lineNo) {
  if (has_message_come()) {
    if (raw_msg_class_ == nullptr) {
      auto rawFactory = apollo::cybertron::message::ProtobufFactory::Instance();
      raw_msg_class_ = rawFactory->GenerateMessageByType(message_type());
    }

    for (auto& iter : children_map_) {
      delete iter.second;
    }

    children_map_.clear();

    if (raw_msg_class_ == nullptr) {
      s->AddStr(0, lineNo++, "Cannot Generate Message by Message Type");
    } else {
      std::ostringstream outStr;
      outStr << std::fixed << std::setprecision(2) << frame_ratio();
      s->AddStr(0, lineNo++, "FrameRatio: ");
      s->AddStr(outStr.str().c_str());

      decltype(channel_message_) channelMsg = CopyMsgPtr();

      if (raw_msg_class_->ParseFromString(channelMsg->message)) {
        int lcount =
            lineCount(*raw_msg_class_, s->Width(), &children_map_, this);
        int pageItemCount = s->Height() - lineNo;
        pages_ = lcount / pageItemCount + 1;
        SplitPages(key);

        PrintMessage(*raw_msg_class_, s, lineNo, 0,
                     page_index_ * pageItemCount);

      } else {
        s->AddStr(0, lineNo++, "Cannot parse the raw message");
      }
    }
  } else {
    s->AddStr(0, lineNo++, "No Message Came");
  }
}
