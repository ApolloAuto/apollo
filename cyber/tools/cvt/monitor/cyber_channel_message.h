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

#ifndef TOOLS_CVT_MONITOR_CYBER_CHANNEL_MESSAGE_H_
#define TOOLS_CVT_MONITOR_CYBER_CHANNEL_MESSAGE_H_

#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "./general_message_base.h"
#include "./renderable_message.h"
#include "cyber/cyber.h"
#include "cyber/time/duration.h"
#include "cyber/time/time.h"

class Screen;

class ChannelMessage : public GeneralMessageBase {
  static double max_frmae_ratio_;

 public:
  static double max_frame_ratio(void) { return max_frmae_ratio_; }

  enum class ErrorCode {
    NewSubClassFailed = -1,
    CreateNodeFailed = -2,
    CreateReaderFailed = -3,
    MessageTypeIsEmptr = -4
  };

  static const char* errCode2Str(ErrorCode errCode) {
    const char* ret = "Unknown Error Code";
    switch (errCode) {
      case ChannelMessage::ErrorCode::NewSubClassFailed:
        ret = "Cannot create Parser Object";
        break;

      case ChannelMessage::ErrorCode::CreateNodeFailed:
        ret = "Cannot create Cyber Node";
        break;

      case ChannelMessage::ErrorCode::CreateReaderFailed:
        ret = "Cannot create Cyber Reader";
        break;

      case ChannelMessage::ErrorCode::MessageTypeIsEmptr:
        ret = "Message Type is Empty";
        break;
    }
    return ret;
  }

  static bool isErrorCode(void* ptr) {
    ErrorCode err = (ErrorCode)(reinterpret_cast<intptr_t>(ptr));
    switch (err) {
      case ErrorCode::NewSubClassFailed:
      case ErrorCode::CreateNodeFailed:
      case ErrorCode::CreateReaderFailed:
      case ErrorCode::MessageTypeIsEmptr:
        return true;

      default: {}
    }
    return false;
  }

  static ErrorCode castPtr2ErrorCode(void* ptr) {
    assert(isErrorCode(ptr));

    return static_cast<ErrorCode>(reinterpret_cast<intptr_t>(ptr));
  }
  static ChannelMessage* castErrorCode2Ptr(ErrorCode errCode) {
    return reinterpret_cast<ChannelMessage*>(static_cast<intptr_t>(errCode));
  }

  explicit ChannelMessage(RenderableMessage* parent = nullptr)
      : GeneralMessageBase(parent),
        is_enabled_(true),
        has_message_come_(false),
        message_type_(),
        frame_counter_(0),
        frame_ratio_(0.0),
        last_time_(apollo::cyber::Time::Now()) {}

  ~ChannelMessage() { channel_node_.reset(); }

  void set_message_type(const std::string& msgTypeName) {
    message_type_ = msgTypeName;
  }
  const std::string& message_type(void) const { return message_type_; }

  bool is_enabled(void) const { return is_enabled_; }
  void set_enabled(bool b) {
    is_enabled_ = b;
    // if (!b) set_has_message_come(false);
  }

  bool has_message_come(void) const { return has_message_come_; }

  double frame_ratio(void) {
    if (!is_enabled_ || !has_message_come()) return 0.0;
    apollo::cyber::Time curTime = apollo::cyber::Time::Now();
    auto deltaTime = curTime - last_time_;

    if (deltaTime.ToNanosecond() > 1000000000) {
      last_time_ = curTime;
      frame_ratio_ = frame_counter_ / deltaTime.ToSecond();
      frame_counter_ = 0;
    }

    if (frame_ratio_ > max_frmae_ratio_) max_frmae_ratio_ = frame_ratio_;

    return (frame_ratio_);
  }

  const std::string& NodeName(void) const { return channel_node_->Name(); }

  void add_reader(const std::string& reader) { DoAdd(readers_, reader); }
  void del_reader(const std::string& reader) { DoDelete(readers_, reader); }

  void add_writer(const std::string& writer) { DoAdd(writers_, writer); }
  void del_writer(const std::string& writer) {
    DoDelete(writers_, writer);
    if (!writers_.size()) {
      set_has_message_come(false);
    }
  }

 protected:
  static void DoDelete(std::vector<std::string>& vec, const std::string& str) {
    for (auto iter = vec.begin(); iter != vec.end(); ++iter) {
      if (*iter == str) {
        vec.erase(iter);
        break;
      }
    }
  }

  static void DoAdd(std::vector<std::string>& vec, const std::string& str) {
    for (auto iter = vec.begin(); iter != vec.end(); ++iter) {
      if (*iter == str) {
        return;
      }
    }

    vec.emplace_back(str);
  }

  void set_has_message_come(bool b) { has_message_come_ = b; }

  bool is_enabled_;
  bool has_message_come_;
  std::string message_type_;
  int frame_counter_;
  double frame_ratio_;
  apollo::cyber::Time last_time_;

  std::unique_ptr<apollo::cyber::Node> channel_node_;

  std::vector<std::string> readers_;
  std::vector<std::string> writers_;
};

template <typename MessageType>
class CyberChannelMessage : public ChannelMessage {
 public:
  explicit CyberChannelMessage(RenderableMessage* parent = nullptr)
      : ChannelMessage(parent) {}

  ~CyberChannelMessage() {
    channel_reader_.reset();
    channel_message_.reset();
  }

  std::string GetChannelName(void) const {
    return channel_reader_->GetChannelName();
  }

 protected:
  void updateRawMessage(const std::shared_ptr<MessageType>& rawMsg) {
    set_has_message_come(true);
    if (!is_enabled_) {
      return;
    }
    ++frame_counter_;
    std::lock_guard<std::mutex> _g(inner_lock_);
    channel_message_.reset();
    channel_message_ = rawMsg;
  }

  std::shared_ptr<MessageType> CopyMsgPtr(void) const {
    decltype(channel_message_) channelMsg;
    {
      std::lock_guard<std::mutex> g(inner_lock_);
      channelMsg = channel_message_;
    }
    return channelMsg;
  }

  std::shared_ptr<MessageType> channel_message_;
  std::shared_ptr<apollo::cyber::Reader<MessageType>> channel_reader_;
  mutable std::mutex inner_lock_;
};

#define ChannelMsgSubFactory(ChannelMsgSubClass, MessageType)              \
  static ChannelMessage* Instance(const std::string& channelName,          \
                                  const std::string& nodeName) {           \
    ChannelMessage* ret = castErrorCode2Ptr(ErrorCode::NewSubClassFailed); \
    ChannelMsgSubClass* subClass = new ChannelMsgSubClass();               \
    if (subClass) {                                                        \
      ret = subClass;                                                      \
      subClass->channel_node_ = apollo::cyber::CreateNode(nodeName);   \
      if (subClass->channel_node_ == nullptr) {                            \
        delete subClass;                                                   \
        subClass = nullptr;                                                \
        ret = castErrorCode2Ptr(ErrorCode::CreateNodeFailed);              \
      } else {                                                             \
        auto callBack =                                                    \
            [subClass](const std::shared_ptr<MessageType>& rawMsg) {       \
              subClass->updateRawMessage(rawMsg);                          \
            };                                                             \
        apollo::cyber::ReaderConfig reader_cfg;                        \
        reader_cfg.channel_name = channelName;                             \
        reader_cfg.pending_queue_size = 20;                                \
        subClass->channel_reader_ =                                        \
            subClass->channel_node_->CreateReader<MessageType>(reader_cfg, \
                                                               callBack);  \
        if (subClass->channel_reader_ == nullptr) {                        \
          subClass->channel_node_.reset();                                 \
          delete subClass;                                                 \
          subClass = nullptr;                                              \
          ret = castErrorCode2Ptr(ErrorCode::CreateReaderFailed);          \
        }                                                                  \
      }                                                                    \
    }                                                                      \
    return ret;                                                            \
  }

#endif  // TOOLS_CVT_MONITOR_CYBER_CHANNEL_MESSAGE_H_
