/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "unknown_message.h"
#include "screen.h"

#include <cybertron/message/raw_message.h>
#include <ncurses.h>
#include <iomanip>
#include <sstream>

namespace{
  constexpr int ReaderWriterOffset = 4;
  int lineCount(const std::string& str)
  {
    int ret = 0;
    for(int i = 0; i < str.length(); ++i)
    {
      if(str.at(i) == '\n')
        ++ret;
    }

    return ret + 1;
  }
}

void UnknownMessage::Render(const Screen *s, int key) {
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

  if (current_state_ == State::ShowDebugString) {
    RenderDebugString(s, key);
  } else {
    RenderInfo(s, key);
  }
}

void UnknownMessage::splitPages(int key)
{
    switch(key){
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

void UnknownMessage::RenderInfo(const Screen *s, int key) {
  int pageItemCount = s->Height() - 3;
  pages_ = (readers_.size() + writers_.size() + 3)/pageItemCount + 1;
  splitPages(key);

  bool hasReader = true;
  std::vector< std::string >* vec = &readers_;

  auto iter = vec->cbegin();
  int y = page_index_ * pageItemCount;
  if(y < vec->size()){
    while (y < page_index_ * pageItemCount) {
      ++iter;
      ++y;
    }
  } else {
    y -= vec->size();
    vec = &writers_;
    iter = vec->cbegin();
    while(y)
    {
      ++iter;
      --y;
    }

    hasReader = false;
  }

  y = 0;

  s->SetCurrentColor(Screen::WHITE_BLACK);

  s->AddStr(0, y++, "ChannelName: ");
  s->AddStr(channel_reader_->GetChannelName().c_str());

  s->AddStr(0, y++, "MessageType: ");
  s->AddStr(message_type().c_str());

  y++;

  if(hasReader)
  {
    s->AddStr(0, y++, "Readers:");
    for (; iter != vec->cend(); ++iter) {
      s->AddStr(ReaderWriterOffset, y++, iter->c_str());
    }

    y++;
    vec = &writers_;
    iter = vec->cbegin();
  }

  s->AddStr(0, y++, "Writers:");
  for (; iter != vec->cend(); ++iter) {
    s->AddStr(ReaderWriterOffset, y++, iter->c_str());
  }

  s->ClearCurrentColor(Screen::WHITE_BLACK);
}

void UnknownMessage::RenderDebugString(const Screen *s, int key) {
  unsigned y = 0;

  s->SetCurrentColor(Screen::WHITE_BLACK);

  s->AddStr(0, y++, "ChannelName: ");
  s->AddStr(channel_reader_->GetChannelName().c_str());

  s->AddStr(0, y++, "MessageType: ");
  s->AddStr(message_type().c_str());
  y++;

  if (has_message_come()) {

    auto rawFactory = apollo::cybertron::message::ProtobufFactory::Instance();
    auto rawMsg = rawFactory->GenerateMessageByType(message_type());

    if (rawMsg == nullptr) {
      s->AddStr(0, y++, "Cannot Generate Message by Message Type");
    } else {
      std::ostringstream outStr;
      outStr << std::fixed << std::setprecision(2) << frame_ratio();
      s->AddStr(0, y++, "FrameRatio: ");
      s->AddStr(outStr.str().c_str());

      decltype(channel_message_) channelMsg = CopyMsgPtr();

      if (rawMsg->ParseFromString(channelMsg->message)) {
        s->AddStr(0, y++, "DebugString:");
        std::string debugStr = rawMsg->DebugString();

        int pageItemCount = s->Height() - 4;
        pages_ = lineCount(debugStr)/pageItemCount + 1;
        splitPages(key);
        int jumpline = page_index_ * pageItemCount;
        const char* ptr = debugStr.c_str();
        while(*ptr != '\0')
        { 
          if(*ptr == '\n') --jumpline;
          if(!jumpline) break;
          ++ptr;
        }

        s->AddStr(0, y++, ptr);
      } else {
        s->AddStr(0, y++, "Cannot parse the raw message");
      }

      delete rawMsg;
    }
  } else {
    s->AddStr(0, y++, "No Message Came");
  }

  s->ClearCurrentColor(Screen::WHITE_BLACK);
}
