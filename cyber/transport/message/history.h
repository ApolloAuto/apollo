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

#ifndef CYBER_TRANSPORT_MESSAGE_HISTORY_H_
#define CYBER_TRANSPORT_MESSAGE_HISTORY_H_

#include <cstdint>
#include <list>
#include <memory>
#include <mutex>
#include <vector>

#include "cyber/common/global_data.h"
#include "cyber/transport/message/history_attributes.h"
#include "cyber/transport/message/message_info.h"

namespace apollo {
namespace cyber {
namespace transport {

template <typename MessageT>
class History {
 public:
  using MessagePtr = std::shared_ptr<MessageT>;
  struct CachedMessage {
    CachedMessage(const MessagePtr& message, const MessageInfo& message_info)
        : msg(message), msg_info(message_info) {}

    MessagePtr msg;
    MessageInfo msg_info;
  };

  explicit History(const HistoryAttributes& attr);
  virtual ~History();

  void Enable() { enabled_ = true; }
  void Disable() { enabled_ = false; }

  void Add(const MessagePtr& msg, const MessageInfo& msg_info);
  void Clear();
  void GetCachedMessage(std::vector<CachedMessage>* msgs) const;
  size_t GetSize() const;

  uint32_t depth() const { return depth_; }
  uint32_t max_depth() const { return max_depth_; }

 private:
  bool enabled_;
  uint32_t depth_;
  uint32_t max_depth_;
  std::list<CachedMessage> msgs_;
  mutable std::mutex msgs_mutex_;
};

template <typename MessageT>
History<MessageT>::History(const HistoryAttributes& attr)
    : enabled_(false), max_depth_(1000) {
  auto& global_conf = common::GlobalData::Instance()->Config();
  if (global_conf.has_transport_conf() &&
      global_conf.transport_conf().has_resource_limit()) {
    max_depth_ =
        global_conf.transport_conf().resource_limit().max_history_depth();
  }

  if (attr.history_policy == proto::QosHistoryPolicy::HISTORY_KEEP_ALL) {
    depth_ = max_depth_;
  } else {
    depth_ = attr.depth;
    if (depth_ > max_depth_) {
      depth_ = max_depth_;
    }
  }
}

template <typename MessageT>
History<MessageT>::~History() {
  Clear();
}

template <typename MessageT>
void History<MessageT>::Add(const MessagePtr& msg,
                            const MessageInfo& msg_info) {
  if (!enabled_) {
    return;
  }
  std::lock_guard<std::mutex> lock(msgs_mutex_);
  msgs_.emplace_back(msg, msg_info);
  while (msgs_.size() > depth_) {
    msgs_.pop_front();
  }
}

template <typename MessageT>
void History<MessageT>::Clear() {
  std::lock_guard<std::mutex> lock(msgs_mutex_);
  msgs_.clear();
}

template <typename MessageT>
void History<MessageT>::GetCachedMessage(
    std::vector<CachedMessage>* msgs) const {
  if (msgs == nullptr) {
    return;
  }

  std::lock_guard<std::mutex> lock(msgs_mutex_);
  msgs->reserve(msgs_.size());
  msgs->insert(msgs->begin(), msgs_.begin(), msgs_.end());
}

template <typename MessageT>
size_t History<MessageT>::GetSize() const {
  std::lock_guard<std::mutex> lock(msgs_mutex_);
  return msgs_.size();
}

}  // namespace transport
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_TRANSPORT_MESSAGE_HISTORY_H_
