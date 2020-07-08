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

#pragma once

#include <memory>
#include <string>

#include "cyber/cyber.h"

class MainWindow;

template <typename T>
using CyberChannelCallback = std::function<void(const std::shared_ptr<T>&)>;

template <typename T>
class CyberChannReader {
 public:
  CyberChannReader(void) : channel_callback_(nullptr), channel_node_(nullptr) {}

  ~CyberChannReader() { CloseChannel(); }

  void CloseChannel(void) {
    if (channel_reader_ != nullptr) {
      channel_reader_.reset();
    }

    if (channel_node_ != nullptr) {
      channel_node_.reset();
    }
  }

  bool InstallCallbackAndOpen(CyberChannelCallback<T> channelCallback,
                              const std::string& channelName,
                              const std::string& nodeName) {
    return InstallCallback(channelCallback) &&
           OpenChannel(channelName, nodeName);
  }

  bool InstallCallback(CyberChannelCallback<T> channelCallback) {
    if (channelCallback != nullptr) {
      channel_callback_ = channelCallback;
      return true;
    } else {
      std::cerr << "Parameter readerCallback is null" << std::endl;
      return false;
    }
  }

  bool OpenChannel(const std::string& channelName,
                   const std::string& nodeName) {
    if (channelName.empty() || nodeName.empty()) {
      std::cerr << "Channel Name or Node Name must be not empty" << std::endl;
      return false;
    }

    if (channel_node_ != nullptr || channel_reader_ != nullptr ||
        !channel_callback_) {
      return false;
    }

    return CreateChannel(channelName, nodeName);
  }

  const std::string& NodeName(void) const { return channel_node_->Name(); }

 private:
  bool CreateChannel(const std::string& channelName,
                     const std::string& nodeName) {
    if (channel_node_ == nullptr) {
      channel_node_ = apollo::cyber::CreateNode(nodeName);
      if (channel_node_ == nullptr) {
        return false;
      }
    }

    channel_reader_ =
        channel_node_->CreateReader<T>(channelName, channel_callback_);

    if (channel_reader_ == nullptr) {
      std::cout << "----------Creat reader failed---------" << std::endl;
      return false;
    }
    return true;
  }

  CyberChannelCallback<T> channel_callback_;
  std::shared_ptr<apollo::cyber::Reader<T>> channel_reader_;
  std::shared_ptr<apollo::cyber::Node> channel_node_;
};
