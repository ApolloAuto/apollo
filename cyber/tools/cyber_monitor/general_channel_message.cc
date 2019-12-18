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

#include "cyber/tools/cyber_monitor/general_channel_message.h"

#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include "cyber/record/record_message.h"
#include "cyber/tools/cyber_monitor/general_message.h"
#include "cyber/tools/cyber_monitor/screen.h"

namespace {
constexpr int ReaderWriterOffset = 4;
using apollo::cyber::record::kGB;
using apollo::cyber::record::kKB;
using apollo::cyber::record::kMB;
}  // namespace

const char* GeneralChannelMessage::errCode2Str(
    GeneralChannelMessage::ErrorCode errCode) {
  const char* ret;
  switch (errCode) {
    case GeneralChannelMessage::ErrorCode::NewSubClassFailed:
      ret = "Cannot Create Parser Object";
      break;

    case GeneralChannelMessage::ErrorCode::CreateNodeFailed:
      ret = "Cannot Create Cyber Node";
      break;

    case GeneralChannelMessage::ErrorCode::CreateReaderFailed:
      ret = "Cannot Create Cyber Reader";
      break;

    case GeneralChannelMessage::ErrorCode::MessageTypeIsEmpty:
      ret = "Message Type is Empty";
      break;

    case GeneralChannelMessage::ErrorCode::ChannelNameOrNodeNameIsEmpty:
      ret = "Channel Name or Node Name is Empty";
      break;

    case GeneralChannelMessage::ErrorCode::NoCloseChannel:
      ret = "No Close Channel";
      break;

    default:
      ret = "Unknown Error Code";
  }
  return ret;
}

bool GeneralChannelMessage::isErrorCode(void* ptr) {
  GeneralChannelMessage::ErrorCode err =
      (GeneralChannelMessage::ErrorCode)(reinterpret_cast<intptr_t>(ptr));
  switch (err) {
    case ErrorCode::NewSubClassFailed:
    case ErrorCode::CreateNodeFailed:
    case ErrorCode::CreateReaderFailed:
    case ErrorCode::MessageTypeIsEmpty:
    case ErrorCode::ChannelNameOrNodeNameIsEmpty:
    case ErrorCode::NoCloseChannel:
      return true;

    default: {}
  }
  return false;
}

double GeneralChannelMessage::frame_ratio(void) {
  if (!is_enabled() || !has_message_come()) {
    return 0.0;
  }
  auto time_now = apollo::cyber::Time::MonoTime();
  auto interval = time_now - time_last_calc_;
  if (interval.ToNanosecond() > 1000000000) {
    int old = frame_counter_;
    while (!frame_counter_.compare_exchange_strong(old, 0)) {
    }
    if (old == 0) {
      return 0.0;
    }
    auto curMsgTime = msg_time_;
    auto deltaTime = curMsgTime - last_time_;
    frame_ratio_ = old / deltaTime.ToSecond();
    last_time_ = curMsgTime;
    time_last_calc_ = time_now;
  }
  return frame_ratio_;
}

GeneralChannelMessage* GeneralChannelMessage::OpenChannel(
    const std::string& channelName) {
  if (channelName.empty() || node_name_.empty()) {
    return castErrorCode2Ptr(ErrorCode::ChannelNameOrNodeNameIsEmpty);
  }
  if (channel_node_ != nullptr || channel_reader_ != nullptr) {
    return castErrorCode2Ptr(ErrorCode::NoCloseChannel);
  }

  channel_node_ = apollo::cyber::CreateNode(node_name_);
  if (channel_node_ == nullptr) {
    return castErrorCode2Ptr(ErrorCode::CreateNodeFailed);
  }

  auto callBack =
      [this](
          const std::shared_ptr<apollo::cyber::message::RawMessage>& rawMsg) {
        updateRawMessage(rawMsg);
      };

  channel_reader_ =
      channel_node_->CreateReader<apollo::cyber::message::RawMessage>(
          channelName, callBack);
  if (channel_reader_ == nullptr) {
    channel_node_.reset();
    return castErrorCode2Ptr(ErrorCode::CreateReaderFailed);
  }
  return this;
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
  pages_ = static_cast<int>(readers_.size() + writers_.size() + lineNo) /
               page_item_count_ +
           1;
  SplitPages(key);

  bool hasReader = true;
  std::vector<std::string>* vec = &readers_;

  auto iter = vec->cbegin();
  unsigned int y = page_index_ * page_item_count_;
  if (y < vec->size()) {
    for (unsigned i = 0; i < y; ++i) {
      ++iter;
    }
  } else {
    y -= static_cast<unsigned int>(vec->size());
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
      outStr << std::fixed << std::setprecision(FrameRatio_Precision)
             << frame_ratio();
      s->AddStr(outStr.str().c_str());

      decltype(channel_message_) channelMsg = CopyMsgPtr();

      if (channelMsg->message.size()) {
        s->AddStr(0, lineNo++, "RawMessage Size: ");
        outStr.str("");
        outStr << channelMsg->message.size() << " Bytes";
        if (channelMsg->message.size() >= kGB) {
          outStr << " (" << static_cast<float>(channelMsg->message.size()) / kGB
                 << " GB)";
        } else if (channelMsg->message.size() >= kMB) {
          outStr << " (" << static_cast<float>(channelMsg->message.size()) / kMB
                 << " MB)";
        } else if (channelMsg->message.size() >= kKB) {
          outStr << " (" << static_cast<float>(channelMsg->message.size()) / kKB
                 << " KB)";
        }
        s->AddStr(outStr.str().c_str());
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
      } else {
        s->AddStr(0, lineNo++, "The size of this raw Message is Zero");
      }
    }
  } else {
    s->AddStr(0, lineNo++, "No Message Came");
  }
}
