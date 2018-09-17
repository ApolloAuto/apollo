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

#include "cybertron_topology_message.h"
#include "channel_msg_factory.h"
#include "cybertron/proto/topology_change.pb.h"
#include "cybertron_channel_message.h"
#include "screen.h"

#include <ncurses.h>
#include <iomanip>
#include <iostream>

constexpr int SecondColumnOffset = 4;

CybertronTopologyMessage::CybertronTopologyMessage()
    : RenderableMessage(),
      second_column_(SecondColumnType::MessageFrameRatio),
      pages_(1),
      page_item_count_(24),
      page_index_(0),
      col1_width_(8),
      all_channels_map_() {}

CybertronTopologyMessage::~CybertronTopologyMessage(void) {
  apollo::cybertron::Shutdown();
  for (auto item : all_channels_map_) {
    if (!ChannelMessage::isErrorCode(item.second)) {
      delete item.second;
    }
  }
}

RenderableMessage* CybertronTopologyMessage::Child(int lineNo) const {
  RenderableMessage* ret = nullptr;
  --lineNo;

  if (lineNo > -1 && lineNo < page_item_count_) {
    int i = 0;

    auto iter = all_channels_map_.cbegin();
    while (i < page_index_ * page_item_count_) {
      ++iter;
      ++i;
    }

    for (i = 0; iter != all_channels_map_.cend(); ++iter) {
      if (i == lineNo) {
        if (!ChannelMessage::isErrorCode(iter->second)) {
          ret = iter->second;
        }
        break;
      }
      ++i;
    }
  }
  return ret;
}

void CybertronTopologyMessage::TopologyChanged(
    const apollo::cybertron::proto::ChangeMsg& changeMsg) {
  const std::string& nodeName = changeMsg.role_attr().node_name();
  const std::string& channelName = changeMsg.role_attr().channel_name();
  const std::string& msgTypeName = changeMsg.role_attr().message_type();

  if ((int)channelName.length() > col1_width_) {
    col1_width_ = channelName.length();
  }

  if (::apollo::cybertron::proto::OperateType::OPT_JOIN ==
      changeMsg.operate_type()) {
    if (ChannelMsgFactory::Instance()->isFromHere(nodeName)) {
      return;
    }

    if (::apollo::cybertron::proto::RoleType::ROLE_WRITER ==
            changeMsg.role_type() ||
        ::apollo::cybertron::proto::RoleType::ROLE_READER ==
            changeMsg.role_type()) {
      if (::apollo::cybertron::proto::ChangeType::CHANGE_CHANNEL ==
          changeMsg.change_type()) {
        auto iter = all_channels_map_.find(channelName);

        if (iter == all_channels_map_.cend()) {
          ChannelMessage* channelMsg =
              ChannelMsgFactory::Instance()->CreateChannelMessage(
                  msgTypeName, channelName);

          if (!ChannelMessage::isErrorCode(channelMsg)) {
            channelMsg->set_parent(this);
            channelMsg->set_message_type(msgTypeName);

            channelMsg->add_reader(channelMsg->NodeName());

            if (::apollo::cybertron::proto::RoleType::ROLE_WRITER ==
                changeMsg.role_type()) {
              channelMsg->add_writer(nodeName);
            } else {
              channelMsg->add_reader(nodeName);
            }
          }

          all_channels_map_[channelName] = channelMsg;
        } else {
          if (!ChannelMessage::isErrorCode(iter->second)) {
            if (::apollo::cybertron::proto::RoleType::ROLE_WRITER ==
                changeMsg.role_type()) {
              iter->second->add_writer(nodeName);
            }
            if (::apollo::cybertron::proto::RoleType::ROLE_READER ==
                changeMsg.role_type()) {
              iter->second->add_reader(nodeName);
            }
          }
        }
      }
    }
  } else {
    if (::apollo::cybertron::proto::ChangeType::CHANGE_CHANNEL ==
        changeMsg.change_type()) {
      auto iter = all_channels_map_.find(channelName);

      if (iter != all_channels_map_.cend() &&
          !ChannelMessage::isErrorCode(iter->second)) {
        if (::apollo::cybertron::proto::RoleType::ROLE_WRITER ==
            changeMsg.role_type()) {
          iter->second->del_writer(nodeName);
        }
        if (::apollo::cybertron::proto::RoleType::ROLE_READER ==
            changeMsg.role_type()) {
          iter->second->del_reader(nodeName);
        }
      }
    }
  }
}

void CybertronTopologyMessage::ChangeState(const Screen* s, int key) {
  switch (key) {
    case 'f':
    case 'F':
      second_column_ = SecondColumnType::MessageFrameRatio;
      break;

    case 't':
    case 'T':
      second_column_ = SecondColumnType::MessageType;
      break;

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

    case ' ':
    {
      ChannelMessage* child = static_cast<ChannelMessage*>(Child(s->highlight_line_no()));
      if(child){
        child->set_enabled(!child->is_enabled());
      }
    }

    default:;
  }
}

void CybertronTopologyMessage::Render(const Screen* s, int key) {
  page_item_count_ = s->Height() - 1;
  pages_ = all_channels_map_.size() / page_item_count_ + 1;
  ChangeState(s, key);

  unsigned y = 0;

  auto iter = all_channels_map_.cbegin();
  while (y < page_index_ * page_item_count_) {
    ++iter;
    ++y;
  }

  y = 0;
  page_item_count_++;

  s->AddStr(0, y, Screen::WHITE_BLACK, "Channels");

  switch (second_column_) {
    case SecondColumnType::MessageType:
      s->AddStr(col1_width_ + SecondColumnOffset, y, Screen::WHITE_BLACK,
                "TypeName");
      break;
    case SecondColumnType::MessageFrameRatio:
      s->AddStr(col1_width_ + SecondColumnOffset, y, Screen::WHITE_BLACK,
                "FrameRatio");
      break;
  }

  ++y;

  Screen::ColorPair color;
  std::ostringstream outStr;

  for (; iter != all_channels_map_.cend() && y < page_item_count_;
       ++iter, ++y) {
    color = Screen::RED_BLACK;

    if (!ChannelMessage::isErrorCode(iter->second)) {
      if (iter->second->is_enabled() && iter->second->has_message_come()) color = Screen::GREEN_BLACK;
    }

    s->SetCurrentColor(color);
    s->AddStr(0, y, iter->first.c_str());

    if (!ChannelMessage::isErrorCode(iter->second)) {
      switch (second_column_) {
        case SecondColumnType::MessageType:
          s->AddStr(col1_width_ + SecondColumnOffset, y,
                    iter->second->message_type().c_str());
          break;
        case SecondColumnType::MessageFrameRatio: {
          outStr.str("");
          outStr << std::fixed << std::setprecision(2)
                 << iter->second->frame_ratio();
          s->AddStr(col1_width_ + SecondColumnOffset, y, outStr.str().c_str());
        } break;
      }
    } else {
      ChannelMessage::ErrorCode errcode =
          ChannelMessage::castPtr2ErrorCode(iter->second);
      s->AddStr(col1_width_ + SecondColumnOffset, y,
                ChannelMessage::errCode2Str(errcode));
    }
    s->ClearCurrentColor(color);
  }
}
