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

#ifndef TOOLS_CVT_MONITOR_GENERAL_CHANNEL_MESSAGE_H_
#define TOOLS_CVT_MONITOR_GENERAL_CHANNEL_MESSAGE_H_

#include <atomic>

#include "cyber/message/raw_message.h"
#include "general_message_base.h"

class CyberTopologyMessage;
class GeneralMessage;

class GeneralChannelMessage : public GeneralMessageBase {
 public:
  enum class ErrorCode {
    NewSubClassFailed = -1,
    CreateNodeFailed = -2,
    CreateReaderFailed = -3,
    MessageTypeIsEmpty = -4,
    ChannelNameOrNodeNameIsEmpty = -5,
    NoCloseChannel = -6
  };

  static const char* errCode2Str(ErrorCode errCode);
  static bool isErrorCode(void* ptr);

  static ErrorCode castPtr2ErrorCode(void* ptr) {
    assert(isErrorCode(ptr));
    return static_cast<ErrorCode>(reinterpret_cast<intptr_t>(ptr));
  }
  static GeneralChannelMessage* castErrorCode2Ptr(ErrorCode errCode) {
    return reinterpret_cast<GeneralChannelMessage*>(
        static_cast<intptr_t>(errCode));
  }

  ~GeneralChannelMessage() {
    channel_node_.reset();
    channel_reader_.reset();
    channel_message_.reset();
    if (raw_msg_class_) {
      delete raw_msg_class_;
      raw_msg_class_ = nullptr;
    }
  }

  std::string GetChannelName(void) const {
    return channel_reader_->GetChannelName();
  }

  void set_message_type(const std::string& msgTypeName) {
    message_type_ = msgTypeName;
  }
  const std::string& message_type(void) const { return message_type_; }

  bool is_enabled(void) const { return channel_reader_ != nullptr; }
  bool has_message_come(void) const { return has_message_come_; }

  double frame_ratio(void) override;

  const std::string& NodeName(void) const { return node_name_; }

  void add_reader(const std::string& reader) { DoAdd(readers_, reader); }
  void del_reader(const std::string& reader) { DoDelete(readers_, reader); }

  void add_writer(const std::string& writer) { DoAdd(writers_, writer); }
  void del_writer(const std::string& writer) {
    DoDelete(writers_, writer);
    if (!writers_.size()) {
      set_has_message_come(false);
    }
  }

  void Render(const Screen* s, int key) override;

  void CloseChannel(void) {
    if (channel_reader_ != nullptr) {
      channel_reader_.reset();
    }

    if (channel_node_ != nullptr) {
      channel_node_.reset();
    }
  }

 private:
  explicit GeneralChannelMessage(const std::string& nodeName,
                                 RenderableMessage* parent = nullptr)
      : GeneralMessageBase(parent),
        current_state_(State::ShowDebugString),
        has_message_come_(false),
        message_type_(),
        frame_counter_(0),
        last_time_(apollo::cyber::Time::MonoTime()),
        msg_time_(last_time_.ToNanosecond() + 1),
        channel_node_(nullptr),
        node_name_(nodeName),
        readers_(),
        writers_(),
        channel_message_(nullptr),
        channel_reader_(nullptr),
        inner_lock_(),
        raw_msg_class_(nullptr) {}

  GeneralChannelMessage(const GeneralChannelMessage&) = delete;
  GeneralChannelMessage& operator=(const GeneralChannelMessage&) = delete;

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

  void updateRawMessage(
      const std::shared_ptr<apollo::cyber::message::RawMessage>& rawMsg) {
    set_has_message_come(true);
    msg_time_ = apollo::cyber::Time::MonoTime();
    ++frame_counter_;
    std::lock_guard<std::mutex> _g(inner_lock_);
    channel_message_.reset();
    channel_message_ = rawMsg;
  }

  std::shared_ptr<apollo::cyber::message::RawMessage> CopyMsgPtr(void) const {
    decltype(channel_message_) channelMsg;
    {
      std::lock_guard<std::mutex> g(inner_lock_);
      channelMsg = channel_message_;
    }
    return channelMsg;
  }

  GeneralChannelMessage* OpenChannel(const std::string& channelName);

  void RenderDebugString(const Screen* s, int key, unsigned lineNo);
  void RenderInfo(const Screen* s, int key, unsigned lineNo);

  void set_has_message_come(bool b) { has_message_come_ = b; }

  enum class State { ShowDebugString, ShowInfo } current_state_;

  bool has_message_come_;
  std::string message_type_;
  std::atomic<int> frame_counter_;
  apollo::cyber::Time last_time_;
  apollo::cyber::Time msg_time_;
  apollo::cyber::Time time_last_calc_ = apollo::cyber::Time::MonoTime();

  std::unique_ptr<apollo::cyber::Node> channel_node_;

  std::string node_name_;

  std::vector<std::string> readers_;
  std::vector<std::string> writers_;

  std::shared_ptr<apollo::cyber::message::RawMessage> channel_message_;
  std::shared_ptr<apollo::cyber::Reader<apollo::cyber::message::RawMessage>>
      channel_reader_;
  mutable std::mutex inner_lock_;

  google::protobuf::Message* raw_msg_class_;

  friend class CyberTopologyMessage;
  friend class GeneralMessage;
};  // GeneralChannelMessage

#endif  // TOOLS_CVT_MONITOR_GENERAL_CHANNEL_MESSAGE_H_
