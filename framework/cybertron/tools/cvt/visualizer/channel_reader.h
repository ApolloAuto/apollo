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

#ifndef CYBERCHANNREADER_H
#define CYBERCHANNREADER_H

#include <QThread>
#include <functional>
#include <memory>

#include <cybertron/cybertron.h>

class MainWindow;

template <typename T>
using CyberChannelCallback = std::function<void(const std::shared_ptr<T>&)>;

template <typename T>
class CyberChannReader : public QThread {
 public:
  explicit CyberChannReader(QObject* parent = nullptr)
      : QThread(parent), channel_callback_(nullptr), channel_node_(nullptr) {}

  ~CyberChannReader() { CloseChannel(); }

  void CloseChannel(void) {
    if (channel_reader_ != nullptr) {
      channel_reader_.reset();
    }

    if (channel_node_ != nullptr) {
      channel_node_.reset();
    }
  }

  bool InstallCallback(CyberChannelCallback<T> channelCallback) {
    if (channelCallback != nullptr) {
      channel_callback_ = channelCallback;
      return true;
    } else {
      return false;
    }
  }

  bool InstallCallbackAndOpen(CyberChannelCallback<T> channelCallback,
                              const std::string& channelName) {
    if (channelCallback == nullptr || channelName.empty()) {
      std::cout << "Parameter readerCallback is null" << std::endl;
      return false;
    }

    channel_callback_ = channelCallback;

    return OpenChannel(channelName);
  }

  bool OpenChannel(const std::string& channelName) {
    if (channel_node_ != nullptr || channel_reader_ != nullptr ||
        !channel_callback_) {
      return false;
    }

    return CreateChannel(channelName);
  }

  const std::string& NodeName(void) const { return channel_node_->Name(); }

  void start(Priority priority = InheritPriority) {
    if (channel_reader_ != nullptr) QThread::start(priority);
  }

 protected:
  virtual void run() override { apollo::cybertron::WaitForShutdown(); }

 private:
  bool CreateChannel(const std::string& channelName) {
    if (channel_node_ == nullptr) {
      channel_node_ = apollo::cybertron::CreateNode("CyberChannReader");
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
  std::shared_ptr<apollo::cybertron::Reader<T>> channel_reader_;
  std::shared_ptr<apollo::cybertron::Node> channel_node_;
};

#endif  // CYBERCHANNREADER_H
