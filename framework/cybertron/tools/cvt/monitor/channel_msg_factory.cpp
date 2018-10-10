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

#include "./channel_msg_factory.h"
#include "./general_channel_message.h"

#include <unistd.h>
#include <string>

ChannelMsgFactory* ChannelMsgFactory::Instance() {
  static ChannelMsgFactory factory;
  return &factory;
}

ChannelMsgFactory::ChannelMsgFactory(void) : pid_(getpid()) {}

bool ChannelMsgFactory::isFromHere(const std::string& nodeName) {
  std::ostringstream outStr;
  outStr << "MonitorReader" << pid_;

  std::string templateName = outStr.str();
  const std::string baseName = nodeName.substr(0, templateName.size());

  return (templateName.compare(baseName) == 0);
}

bool ChannelMsgFactory::SetDefaultChildFactory(
    const std::string& defautlMsgType) {
  auto iter = general_factory_.find(defautlMsgType);
  if (iter == general_factory_.cend()) {
    return false;
  }

  default_child_factory_ = iter;

  return true;
}

ChannelMessage* ChannelMsgFactory::CreateChannelMessage(
    const std::string& msgTypeName, const std::string& channelName) {
  static int index = 0;

  std::ostringstream outStr;
  outStr << "MonitorReader" << pid_ << '-' << index++;

  ChannelMessage* ret = ChannelMessage::castErrorCode2Ptr(
      ChannelMessage::ErrorCode::MessageTypeIsEmptr);
  if (!msgTypeName.empty()) {
    auto iter = general_factory_.find(msgTypeName);
    if (iter != general_factory_.cend()) {
      ret = iter->second(channelName, outStr.str());
    } else {
      if (default_child_factory_ != general_factory_.cend()) {
        ret = default_child_factory_->second(channelName, outStr.str());
      }
    }
  }
  return ret;
}
