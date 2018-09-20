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
}  // namespace

RenderableMessage* GeneralMessage::Child(int lineNo) const {
  return GeneralMessageBase::Child(lineNo);
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

    clear();

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
            lineCount(*raw_msg_class_, s->Width());
        int pageItemCount = s->Height() - lineNo;
        pages_ = lcount / pageItemCount + 1;
        SplitPages(key);

        PrintMessage(this, *raw_msg_class_, s, lineNo, 0,
                     page_index_ * pageItemCount);

      } else {
        s->AddStr(0, lineNo++, "Cannot parse the raw message");
      }
    }
  } else {
    s->AddStr(0, lineNo++, "No Message Came");
  }
}
