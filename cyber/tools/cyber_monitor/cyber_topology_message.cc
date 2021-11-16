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

#include "cyber/tools/cyber_monitor/cyber_topology_message.h"

#include <iomanip>
#include <iostream>

#include "cyber/proto/role_attributes.pb.h"
#include "cyber/proto/topology_change.pb.h"

#include "cyber/message/message_traits.h"
#include "cyber/tools/cyber_monitor/general_channel_message.h"
#include "cyber/tools/cyber_monitor/screen.h"

constexpr int SecondColumnOffset = 4;

CyberTopologyMessage::CyberTopologyMessage(const std::string& channel)
    : RenderableMessage(nullptr, 1),
      second_column_(SecondColumnType::MessageFrameRatio),
      pid_(getpid()),
      col1_width_(8),
      specified_channel_(channel),
      all_channels_map_() {}

CyberTopologyMessage::~CyberTopologyMessage(void) {
  for (auto item : all_channels_map_) {
    if (!GeneralChannelMessage::IsErrorCode(item.second)) {
      delete item.second;
    }
  }
}

bool CyberTopologyMessage::IsFromHere(const std::string& node_name) {
  std::ostringstream out_str;
  out_str << "MonitorReader" << pid_;

  std::string templateName = out_str.str();
  const std::string baseName = node_name.substr(0, templateName.size());

  return (templateName.compare(baseName) == 0);
}

RenderableMessage* CyberTopologyMessage::Child(int line_no) const {
  RenderableMessage* ret = nullptr;
  auto iter = FindChild(line_no);
  if (iter != all_channels_map_.cend() &&
      !GeneralChannelMessage::IsErrorCode(iter->second) &&
      iter->second->is_enabled()) {
    ret = iter->second;
  }
  return ret;
}

std::map<std::string, GeneralChannelMessage*>::const_iterator
CyberTopologyMessage::FindChild(int line_no) const {
  --line_no;

  std::map<std::string, GeneralChannelMessage*>::const_iterator ret =
      all_channels_map_.cend();

  if (line_no > -1 && line_no < page_item_count_) {
    int i = 0;

    auto iter = all_channels_map_.cbegin();
    while (i < page_index_ * page_item_count_) {
      ++iter;
      ++i;
    }

    for (i = 0; iter != all_channels_map_.cend(); ++iter) {
      if (i == line_no) {
        ret = iter;
        break;
      }
      ++i;
    }
  }
  return ret;
}

void CyberTopologyMessage::TopologyChanged(
    const apollo::cyber::proto::ChangeMsg& changeMsg) {
  if (::apollo::cyber::proto::OperateType::OPT_JOIN ==
      changeMsg.operate_type()) {
    bool isWriter = true;
    if (::apollo::cyber::proto::RoleType::ROLE_READER == changeMsg.role_type())
      isWriter = false;
    AddReaderWriter(changeMsg.role_attr(), isWriter);
  } else {
    auto iter = all_channels_map_.find(changeMsg.role_attr().channel_name());

    if (iter != all_channels_map_.cend() &&
        !GeneralChannelMessage::IsErrorCode(iter->second)) {
      const std::string& node_name = changeMsg.role_attr().node_name();
      if (::apollo::cyber::proto::RoleType::ROLE_WRITER ==
          changeMsg.role_type()) {
        iter->second->del_writer(node_name);
      } else {
        iter->second->del_reader(node_name);
      }
    }
  }
}

void CyberTopologyMessage::AddReaderWriter(
    const apollo::cyber::proto::RoleAttributes& role, bool isWriter) {
  const std::string& channel_name = role.channel_name();

  if (!specified_channel_.empty() && specified_channel_ != channel_name) {
    return;
  }

  if (static_cast<int>(channel_name.length()) > col1_width_) {
    col1_width_ = static_cast<int>(channel_name.length());
  }

  const std::string& node_name = role.node_name();
  if (IsFromHere(node_name)) {
    return;
  }

  GeneralChannelMessage* channel_msg = nullptr;
  const std::string& msgTypeName = role.message_type();
  auto iter = all_channels_map_.find(channel_name);
  if (iter == all_channels_map_.cend()) {
    static int index = 0;

    std::ostringstream out_str;
    out_str << "MonitorReader" << pid_ << '-' << index++;

    channel_msg = new GeneralChannelMessage(out_str.str(), this);

    if (channel_msg != nullptr) {
      if (!GeneralChannelMessage::IsErrorCode(
              channel_msg->OpenChannel(channel_name))) {
        channel_msg->set_message_type(msgTypeName);
        channel_msg->add_reader(channel_msg->NodeName());
      }
    } else {
      channel_msg = GeneralChannelMessage::CastErrorCode2Ptr(
          GeneralChannelMessage::ErrorCode::NewSubClassFailed);
    }
    all_channels_map_[channel_name] = channel_msg;
  } else {
    channel_msg = iter->second;
  }

  if (!GeneralChannelMessage::IsErrorCode(channel_msg)) {
    if (isWriter) {
      if (msgTypeName != apollo::cyber::message::MessageType<
                             apollo::cyber::message::RawMessage>()) {
        channel_msg->set_message_type(msgTypeName);
      }

      channel_msg->add_writer(node_name);
    } else {
      channel_msg->add_reader(node_name);
    }
  }
}

void CyberTopologyMessage::ChangeState(const Screen* s, int key) {
  switch (key) {
    case 'f':
    case 'F':
      second_column_ = SecondColumnType::MessageFrameRatio;
      break;

    case 't':
    case 'T':
      second_column_ = SecondColumnType::MessageType;
      break;

    case ' ': {
      auto iter = FindChild(*line_no());
      if (!GeneralChannelMessage::IsErrorCode(iter->second)) {
        GeneralChannelMessage* child = iter->second;
        if (child->is_enabled()) {
          child->CloseChannel();
        } else {
          GeneralChannelMessage* ret = child->OpenChannel(iter->first);
          if (GeneralChannelMessage::IsErrorCode(ret)) {
            delete child;
            all_channels_map_[iter->first] = ret;
          } else {
            child->add_reader(child->NodeName());
          }
        }
      }
    }

    default: {
    }
  }
}

int CyberTopologyMessage::Render(const Screen* s, int key) {
  page_item_count_ = s->Height() - 1;
  pages_ = static_cast<int>(all_channels_map_.size()) / page_item_count_ + 1;
  ChangeState(s, key);
  SplitPages(key);

  s->AddStr(0, 0, Screen::WHITE_BLACK, "Channels");
  switch (second_column_) {
    case SecondColumnType::MessageType:
      s->AddStr(col1_width_ + SecondColumnOffset, 0, Screen::WHITE_BLACK,
                "TypeName");
      break;
    case SecondColumnType::MessageFrameRatio:
      s->AddStr(col1_width_ + SecondColumnOffset, 0, Screen::WHITE_BLACK,
                "FrameRatio");
      break;
  }

  auto iter = all_channels_map_.cbegin();
  register int tmp = page_index_ * page_item_count_;
  register int line = 0;
  while (line < tmp) {
    ++iter;
    ++line;
  }

  Screen::ColorPair color;
  std::ostringstream out_str;

  tmp = page_item_count_ + 1;
  for (line = 1; iter != all_channels_map_.cend() && line < tmp;
       ++iter, ++line) {
    color = Screen::RED_BLACK;

    if (!GeneralChannelMessage::IsErrorCode(iter->second)) {
      if (iter->second->has_message_come()) {
        if (iter->second->is_enabled()) {
          color = Screen::GREEN_BLACK;
        } else {
          color = Screen::YELLOW_BLACK;
        }
      }
    }

    s->SetCurrentColor(color);
    s->AddStr(0, line, iter->first.c_str());

    if (!GeneralChannelMessage::IsErrorCode(iter->second)) {
      switch (second_column_) {
        case SecondColumnType::MessageType:
          s->AddStr(col1_width_ + SecondColumnOffset, line,
                    iter->second->message_type().c_str());
          break;
        case SecondColumnType::MessageFrameRatio: {
          out_str.str("");
          out_str << std::fixed << std::setprecision(FrameRatio_Precision)
                  << iter->second->frame_ratio();
          s->AddStr(col1_width_ + SecondColumnOffset, line,
                    out_str.str().c_str());
        } break;
      }
    } else {
      GeneralChannelMessage::ErrorCode errcode =
          GeneralChannelMessage::CastPtr2ErrorCode(iter->second);
      s->AddStr(col1_width_ + SecondColumnOffset, line,
                GeneralChannelMessage::ErrCode2Str(errcode));
    }
    s->ClearCurrentColor();
  }

  return line;
}
