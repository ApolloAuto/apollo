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

#ifndef CYBERTRON_NODE_READER_BASE_H_
#define CYBERTRON_NODE_READER_BASE_H_

#include <atomic>
#include <memory>
#include <string>
#include <unordered_map>

#include "cybertron/common/global_data.h"
#include "cybertron/common/macros.h"
#include "cybertron/common/util.h"
#include "cybertron/transport/transport.h"
#include "cybertron/event/perf_event_cache.h"

namespace apollo {
namespace cybertron {

using apollo::cybertron::common::GlobalData;
using apollo::cybertron::event::PerfEventCache;

class ReaderBase {
 public:
  explicit ReaderBase(const proto::RoleAttributes& role_attr)
      : role_attr_(role_attr), init_(false) {
    if (!role_attr_.has_host_name()) {
      role_attr_.set_host_name(common::GlobalData::Instance()->HostName());
    }
    if (!role_attr_.has_process_id()) {
      role_attr_.set_process_id(common::GlobalData::Instance()->ProcessId());
    }
  }
  virtual ~ReaderBase() {}

  virtual bool Init() = 0;
  virtual void Shutdown() = 0;
  const std::string& GetChannelName() const {
    return role_attr_.channel_name();
  }

  const uint64_t ChannelId() const { return role_attr_.channel_id(); }

  const proto::QosProfile& QosProfile() const {
    return role_attr_.qos_profile();
  }

  bool inited() const { return init_.load(); }

 protected:
  proto::RoleAttributes role_attr_;
  std::atomic<bool> init_;
};

template <typename MessageT>
class ReaderManager {
 public:
  ~ReaderManager() { lower_reach_map_.clear(); }

  auto GetReader(const proto::RoleAttributes& role_attr) ->
      typename std::shared_ptr<transport::LowerReach<MessageT>>;

 private:
  std::unordered_map<std::string,
                     typename std::shared_ptr<transport::LowerReach<MessageT>>>
      lower_reach_map_;
  std::mutex lower_reach_map_mutex_;

  DECLARE_SINGLETON(ReaderManager<MessageT>)
};

template <typename MessageT>
ReaderManager<MessageT>::ReaderManager() {}

template <typename MessageT>
auto ReaderManager<MessageT>::GetReader(const proto::RoleAttributes& role_attr)
    -> typename std::shared_ptr<transport::LowerReach<MessageT>> {
  std::lock_guard<std::mutex> lock(lower_reach_map_mutex_);
  // because multi reader for one channel will write datacache multi times,
  // so reader for datacache we use map to keep one instance for per channel
  const std::string& channel_name = role_attr.channel_name();
  if (lower_reach_map_.count(channel_name) == 0) {
    lower_reach_map_[channel_name] =
        transport::Transport::CreateLowerReach<MessageT>(
            role_attr, [](const std::shared_ptr<MessageT>& msg,
                          const transport::MessageInfo& msg_info,
                          const proto::RoleAttributes& reader_attr) {
              (void)msg_info;
              (void)reader_attr;
              PerfEventCache::Instance()->AddTransportEvent(2, reader_attr.channel_id(), msg_info.seq_num());
              data::DataDispatcher<MessageT>::Instance()->Dispatch(
                  reader_attr.channel_id(), msg);
              PerfEventCache::Instance()->AddTransportEvent(3, reader_attr.channel_id(), msg_info.seq_num());

            });
  }
  return lower_reach_map_[channel_name];
}

}  // namespace cybertron
}  // namespace apollo

#endif  // CYBERTRON_NODE_READER_BASE_H_
