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

#ifndef TOOLS_CVT_MONITOR_CHANNEL_MSG_FACTORY_H_
#define TOOLS_CVT_MONITOR_CHANNEL_MSG_FACTORY_H_

#include <functional>
#include <iostream>
#include <map>
#include <string>

class ChannelMessage;

using CreatorFunction =
    std::function<ChannelMessage*(const std::string&, const std::string&)>;

class ChannelMsgFactory final {
 public:
  static ChannelMsgFactory* Instance(void);

  ~ChannelMsgFactory() {}

  bool RegisterChildFactory(const std::string& msgTypeName, CreatorFunction f) {
    if (msgTypeName.empty() || f == nullptr) {
      return false;
    }
    auto iter = general_factory_.find(msgTypeName);

    if (iter != general_factory_.cend()) {
      return false;
    }

    general_factory_[msgTypeName] = f;

    return true;
  }

  void RemoveChildFactory(const std::string& msgTypeName) {
    if (msgTypeName.empty()) {
      return;
    }
    auto iter = general_factory_.find(msgTypeName);
    if (iter != general_factory_.cend()) {
      general_factory_.erase(iter);
    }
  }

  bool SetDefaultChildFactory(const std::string& defautlMsgType);

  ChannelMessage* CreateChannelMessage(const std::string& msgTypeName,
                                       const std::string& channelName);

  bool isFromHere(const std::string& nodeName);

 private:
  ChannelMsgFactory(void);
  ChannelMsgFactory(const ChannelMsgFactory& other) = delete;
  ChannelMsgFactory& operator=(const ChannelMsgFactory&) = delete;

  int pid_;
  std::map<std::string, CreatorFunction>::const_iterator default_child_factory_;
  std::map<std::string, CreatorFunction> general_factory_;
};

#endif  // TOOLS_CVT_MONITOR_CHANNEL_MSG_FACTORY_H_
