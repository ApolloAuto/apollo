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

#include "./general_channel_message.h"
#include "./general_message.h"
#include "./screen.h"

#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace {
constexpr int ReaderWriterOffset = 4;
}  // namespace

RenderableMessage* GeneralChannelMessage::Child(int lineNo) const {
  return GeneralMessageBase::Child(lineNo);
}

void GeneralChannelMessage::Render(const Screen* s, int key) {
  switch (key) {
    case 'b':
    case 'B':
      current_state_ = State::ShowDebugString;
      break;

    case 'i':
    case 'I':
      current_state_ = State::ShowInfo;
      break;

    default: {}
  }

  clear();

  unsigned lineNo = 0;

  s->SetCurrentColor(Screen::WHITE_BLACK);
  s->AddStr(0, lineNo++, "ChannelName: ");
  s->AddStr(channel_reader_->GetChannelName().c_str());

  s->AddStr(0, lineNo++, "MessageType: ");
  s->AddStr(message_type().c_str());

  if (is_enabled()) {
    switch (current_state_) {
      case State::ShowDebugString:
        RenderDebugString(s, key, lineNo);
        break;
      case State::ShowInfo:
        RenderInfo(s, key, lineNo);
        break;
    }
  } else {
    s->AddStr(0, lineNo++, "Channel has been closed");
  }
  s->ClearCurrentColor();
}

void GeneralChannelMessage::RenderInfo(const Screen* s, int key,
                                       unsigned lineNo) {
  page_item_count_ = s->Height() - lineNo;
  pages_ = (readers_.size() + writers_.size() + lineNo) / page_item_count_ + 1;
  SplitPages(key);

  bool hasReader = true;
  std::vector<std::string>* vec = &readers_;

  auto iter = vec->cbegin();
  unsigned y = page_index_ * page_item_count_;
  if (y < vec->size()) {
    for (unsigned i = 0; i < y; ++i) {
      ++iter;
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

void GeneralChannelMessage::RenderDebugString(const Screen* s, int key,
                                              unsigned lineNo) {
  if (has_message_come()) {
    if (raw_msg_class_ == nullptr) {
      auto rawFactory = apollo::cyber::message::ProtobufFactory::Instance();
      raw_msg_class_ = rawFactory->GenerateMessageByType(message_type());
    }

    if (raw_msg_class_ == nullptr) {
      s->AddStr(0, lineNo++, "Cannot Generate Message by Message Type");
    } else {
      s->AddStr(0, lineNo++, "FrameRatio: ");

      std::ostringstream outStr;
      outStr << std::fixed << std::setprecision(2) << frame_ratio();
      s->AddStr(outStr.str().c_str());

      decltype(channel_message_) channelMsg = CopyMsgPtr();

      if (raw_msg_class_->ParseFromString(channelMsg->message)) {
        int lcount = lineCount(*raw_msg_class_, s->Width());
        page_item_count_ = s->Height() - lineNo;
        pages_ = lcount / page_item_count_ + 1;
        SplitPages(key);
        int jumpLines = page_index_ * page_item_count_;
        jumpLines <<= 2;
        jumpLines /= 5;
        GeneralMessageBase::PrintMessage(this, *raw_msg_class_, jumpLines, s,
                                         lineNo, 0);
      } else {
        s->AddStr(0, lineNo++, "Cannot parse the raw message");
      }
    }
  } else {
    s->AddStr(0, lineNo++, "No Message Came");
  }
}
