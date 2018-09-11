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

#ifndef CYBERTRONCHANNELMESSAGE_H
#define CYBERTRONCHANNELMESSAGE_H

#include <cybertron/cybertron.h>
#include <fstream>
#include <mutex>
#include "cybertron/time/duration.h"
#include "cybertron/time/time.h"
#include "renderable_message.h"

class Screen;

class ChannelMessage : public RenderableMessage {
 public:
  enum class ErrorCode {
    NewSubClassFailed = -1,
    CreateNodeFailed = -2,
    CreateReaderFailed = -3,
    MessageTypeIsEmptr = -4
  };

  static const char* errCode2Str(ErrorCode errCode) {
    switch (errCode) {
      case ChannelMessage::ErrorCode::NewSubClassFailed:
        return "Cannot create Parser Object";

      case ChannelMessage::ErrorCode::CreateNodeFailed:
        return "Cannot create Cybertron Node";

      case ChannelMessage::ErrorCode::CreateReaderFailed:
        return "Cannot create Cybertron Reader";

      case ChannelMessage::ErrorCode::MessageTypeIsEmptr:
        return "Message Type is Empty";
    }
  }

  static bool isErrorCode(void* ptr) {
    ErrorCode err = (ErrorCode)(reinterpret_cast<intptr_t>(ptr));
    switch (err) {
      case ErrorCode::NewSubClassFailed:
      case ErrorCode::CreateNodeFailed:
      case ErrorCode::CreateReaderFailed:
      case ErrorCode::MessageTypeIsEmptr:
        return true;

      default:;
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
      : RenderableMessage(parent),
        has_message_come_(false),
        message_type_(),
        frame_counter_(0),
        frame_ratio_(0.0),
        last_time_(apollo::cybertron::Time::Now()) {}

  ~ChannelMessage() { channel_node_.reset(); }

  void set_message_type(const std::string& msgTypeName) {
    message_type_ = msgTypeName;
  }
  const std::string& message_type(void) const { return message_type_; }

  bool has_message_come(void) const { return has_message_come_; }
  void set_has_message_come(bool b){ has_message_come_ = b; }

  double frame_ratio(void) {
    apollo::cybertron::Time curTime = apollo::cybertron::Time::Now();
    auto deltaTime = curTime - last_time_;

    if (deltaTime.ToNanosecond() > 1000000000) {
      last_time_ = curTime;
      frame_ratio_ = frame_counter_ / deltaTime.ToSecond();
      frame_counter_ = 0;
    }

    return (frame_ratio_);
  }

  const std::string& NodeName(void) const { return channel_node_->Name(); }

  void addReader(const std::string& reader) { readers_.push_back(reader); }
  void delReader(const std::string& reader) { DoDelete(readers_, reader); }

  void addWriter(const std::string& writer) { writers_.push_back(writer); }
  void delWriter(const std::string& writer) { 
    DoDelete(writers_, writer); 
    if(!writers_.size()){
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

  bool has_message_come_;
  std::string message_type_;
  int frame_counter_;
  double frame_ratio_;
  apollo::cybertron::Time last_time_;

  std::unique_ptr<apollo::cybertron::Node> channel_node_;

  std::vector<std::string> readers_;
  std::vector<std::string> writers_;
};

template <typename MessageType>
class CybertronChannelMessage : public ChannelMessage {
 public:
  explicit CybertronChannelMessage(RenderableMessage* parent = nullptr)
      : ChannelMessage(parent) {}

  ~CybertronChannelMessage() {
    channel_reader_.reset();
    channel_message_.reset();
  }

 protected:
  void updateRawMessage(const std::shared_ptr<MessageType>& rawMsg) {
    ++frame_counter_;
    std::lock_guard<std::mutex> _g(inner_lock_);
    set_has_message_come(true);
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
  std::shared_ptr<apollo::cybertron::Reader<MessageType>> channel_reader_;
  mutable std::mutex inner_lock_;
};

#define RegisterChannelMsgClass(ChannelMsgSubClass, MessageType)              \
  static ChannelMessage* Instance(const std::string& channelName,             \
                                  const std::string& nodeName) {              \
    ChannelMessage* ret = castErrorCode2Ptr(ErrorCode::NewSubClassFailed);    \
    ChannelMsgSubClass* subClass = new ChannelMsgSubClass();                  \
    if (subClass) {                                                           \
      ret = subClass;                                                         \
      subClass->channel_node_ = apollo::cybertron::CreateNode(nodeName);      \
      if (subClass->channel_node_ == nullptr) {                               \
        delete subClass;                                                      \
        subClass = nullptr;                                                   \
        ret = castErrorCode2Ptr(ErrorCode::CreateNodeFailed);                 \
      } else {                                                                \
        auto callBack =                                                       \
            [subClass](const std::shared_ptr<MessageType>& rawMsg) {          \
              subClass->updateRawMessage(rawMsg);                             \
            };                                                                \
        subClass->channel_reader_ =                                           \
            subClass->channel_node_->CreateReader<MessageType>(channelName,   \
                                                               callBack);     \
        if (subClass->channel_reader_ == nullptr) {                           \
          subClass->channel_node_.reset();                                    \
          delete subClass;                                                    \
          subClass = nullptr;                                                 \
          ret = castErrorCode2Ptr(ErrorCode::CreateReaderFailed);             \
        }                                                                     \
      }                                                                       \
    }                                                                         \
    return ret;                                                               \
  }

#define BegDefineChannelMsgSubClass(SubClassName, MessageType)              \
  class SubClassName : public CybertronChannelMessage<MessageType> {        \
   public:                                                                  \
    RegisterChannelMsgClass(SubClassName, MessageType) virtual void Render( \
        const Screen* s, int key) override;                                 \
    virtual ~SubClassName() {}                                              \
                                                                            \
   private:                                                                 \
  explicit SubClassName(RenderableMessage* parent = nullptr)                \
      : CybertronChannelMessage<MessageType>(parent)

#define EndDefineChannelMsgSubClass(SubClassName) } /* SubClassName */

#endif  // CYBERTRONCHANNELMESSAGE_H
