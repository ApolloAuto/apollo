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

#ifndef CYBER_NODE_READER_BASE_H_
#define CYBER_NODE_READER_BASE_H_

#include <atomic>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "cyber/common/macros.h"
#include "cyber/common/util.h"
#include "cyber/event/perf_event_cache.h"
#include "cyber/transport/transport.h"

namespace apollo {
namespace cyber {

using apollo::cyber::common::GlobalData;
using apollo::cyber::event::PerfEventCache;
using apollo::cyber::event::TransPerf;

/**
 * @class ReaderBase
 * @brief Base Class for Reader
 * Reader is identified by one apollo::cyber::proto::RoleAttribute,
 * it contains the channel_name, channel_id that we subscribe,
 * and host_name, process_id and node that we are located,
 * and qos that describes our transportation quality.
 */
class ReaderBase {
 public:
  explicit ReaderBase(const proto::RoleAttributes& role_attr)
      : role_attr_(role_attr), init_(false) {}
  virtual ~ReaderBase() {}

  /**
   * @brief Init the Reader object
   *
   * @return true if init successfully
   * @return false if init failed
   */
  virtual bool Init() = 0;

  /**
   * @brief Shutdown the Reader object
   */
  virtual void Shutdown() = 0;

  /**
   * @brief Clear local data
   */
  virtual void ClearData() = 0;

  /**
   * @brief Get stored data
   */
  virtual void Observe() = 0;

  /**
   * @brief Query whether the Reader has data to be handled
   *
   * @return true if data container is empty
   * @return false if data container has data
   */
  virtual bool Empty() const = 0;

  /**
   * @brief Query whether we have received data since last clear
   *
   * @return true if the reader has received data
   * @return false if the reader has not received data
   */
  virtual bool HasReceived() const = 0;

  /**
   * @brief Get time interval of since last receive message
   *
   * @return double seconds delay
   */
  virtual double GetDelaySec() const = 0;

  /**
   * @brief Get the value of pending queue size
   *
   * @return uint32_t result value
   */
  virtual uint32_t PendingQueueSize() const = 0;

  /**
   * @brief Query is there any writer that publish the subscribed channel
   *
   * @return true if there is at least one Writer publish the channel
   * @return false if there is no Writer publish the channel
   */
  virtual bool HasWriter() { return false; }

  /**
   * @brief Get all writers pushlish the channel we subscribes
   *
   * @param writers result RoleAttributes vector
   */
  virtual void GetWriters(std::vector<proto::RoleAttributes>* writers) {}

  /**
   * @brief Get Reader's Channel name
   *
   * @return const std::string& channel name
   */
  const std::string& GetChannelName() const {
    return role_attr_.channel_name();
  }

  /**
   * @brief Get Reader's Channel id
   *
   * @return uint64_t channel id
   */
  uint64_t ChannelId() const { return role_attr_.channel_id(); }

  /**
   * @brief Get qos profile. You can see qos description
   *
   * @return const proto::QosProfile& result qos
   */
  const proto::QosProfile& QosProfile() const {
    return role_attr_.qos_profile();
  }

  /**
   * @brief Query whether the Reader is initialized
   *
   * @return true if the Reader has been inited
   * @return false if the Reader has not been inited
   */
  bool IsInit() const { return init_.load(); }

 protected:
  proto::RoleAttributes role_attr_;
  std::atomic<bool> init_;
};

/**
 * @brief One Channel is related to one Receiver.
 * ReceiverManager is in charge of attaching one Receiver to its responding
 * Channel. We pass a DataDispatche's callback func to this Receiver so when a
 * message is received, it will be push to the `ChannelBuffer`, and
 * `DataVisitor` will `Fetch` data and pass to `Reader`'s callback func
 *
 * @tparam MessageT Message Type.
 */
template <typename MessageT>
class ReceiverManager {
 public:
  ~ReceiverManager() { receiver_map_.clear(); }

  /**
   * @brief Get the Receiver object
   *
   * @param role_attr the attribute that the Receiver has
   * @return std::shared_ptr<transport::Receiver<MessageT>> result Receiver
   */
  auto GetReceiver(const proto::RoleAttributes& role_attr) ->
      typename std::shared_ptr<transport::Receiver<MessageT>>;

 private:
  std::unordered_map<std::string,
                     typename std::shared_ptr<transport::Receiver<MessageT>>>
      receiver_map_;
  std::mutex receiver_map_mutex_;

  DECLARE_SINGLETON(ReceiverManager<MessageT>)
};

/**
 * @brief Construct a new Receiver Manager< Message T>:: Receiver Manager object
 *
 * @tparam MessageT param
 */
template <typename MessageT>
ReceiverManager<MessageT>::ReceiverManager() {}

template <typename MessageT>
auto ReceiverManager<MessageT>::GetReceiver(
    const proto::RoleAttributes& role_attr) ->
    typename std::shared_ptr<transport::Receiver<MessageT>> {
  std::lock_guard<std::mutex> lock(receiver_map_mutex_);
  // because multi reader for one channel will write datacache multi times,
  // so reader for datacache we use map to keep one instance for per channel
  const std::string& channel_name = role_attr.channel_name();
  if (receiver_map_.count(channel_name) == 0) {
    receiver_map_[channel_name] =
        transport::Transport::Instance()->CreateReceiver<MessageT>(
            role_attr, [](const std::shared_ptr<MessageT>& msg,
                          const transport::MessageInfo& msg_info,
                          const proto::RoleAttributes& reader_attr) {
              (void)msg_info;
              (void)reader_attr;
              PerfEventCache::Instance()->AddTransportEvent(
                  TransPerf::TRANS_TO, reader_attr.channel_id(),
                  msg_info.seq_num());
              data::DataDispatcher<MessageT>::Instance()->Dispatch(
                  reader_attr.channel_id(), msg);
              PerfEventCache::Instance()->AddTransportEvent(
                  TransPerf::WRITE_NOTIFY, reader_attr.channel_id(),
                  msg_info.seq_num());
            });
  }
  return receiver_map_[channel_name];
}

}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_NODE_READER_BASE_H_
