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

#ifndef CYBER_NODE_NODE_CHANNEL_IMPL_H_
#define CYBER_NODE_NODE_CHANNEL_IMPL_H_

#include <memory>
#include <string>

#include "cyber/proto/run_mode_conf.pb.h"

#include "cyber/blocker/intra_reader.h"
#include "cyber/blocker/intra_writer.h"
#include "cyber/common/global_data.h"
#include "cyber/message/message_traits.h"
#include "cyber/node/reader.h"
#include "cyber/node/writer.h"

namespace apollo {
namespace cyber {

class Node;

struct ReaderConfig {  ///< configurations for a Reader
  ReaderConfig() {
    qos_profile.set_history(proto::QosHistoryPolicy::HISTORY_KEEP_LAST);
    qos_profile.set_depth(1);
    qos_profile.set_mps(0);
    qos_profile.set_reliability(
        proto::QosReliabilityPolicy::RELIABILITY_RELIABLE);
    qos_profile.set_durability(proto::QosDurabilityPolicy::DURABILITY_VOLATILE);

    pending_queue_size = DEFAULT_PENDING_QUEUE_SIZE;
  }
  ReaderConfig(const ReaderConfig& other)
      : channel_name(other.channel_name),
        qos_profile(other.qos_profile),
        pending_queue_size(other.pending_queue_size) {}

  std::string channel_name;       //< channel reads
  proto::QosProfile qos_profile;  //< the qos configuration
  /**
   * @brief configuration for responding ChannelBuffer.
   * Older messages will dropped if you have no time to handle
   */
  uint32_t pending_queue_size;
};

/**
 * @class NodeChannelImpl
 * @brief The implementation for Node to create Objects connected by Channels.
 * e.g. Channel Reader and Writer
 */
class NodeChannelImpl {
  friend class Node;

 public:
  using NodeManagerPtr = std::shared_ptr<service_discovery::NodeManager>;

  /**
   * @brief Construct a new Node Channel Impl object
   *
   * @param node_name node name
   */
  explicit NodeChannelImpl(const std::string& node_name)
      : is_reality_mode_(true), node_name_(node_name) {
    node_attr_.set_host_name(common::GlobalData::Instance()->HostName());
    node_attr_.set_host_ip(common::GlobalData::Instance()->HostIp());
    node_attr_.set_process_id(common::GlobalData::Instance()->ProcessId());
    node_attr_.set_node_name(node_name);
    uint64_t node_id = common::GlobalData::RegisterNode(node_name);
    node_attr_.set_node_id(node_id);

    is_reality_mode_ = common::GlobalData::Instance()->IsRealityMode();

    if (is_reality_mode_) {
      node_manager_ =
          service_discovery::TopologyManager::Instance()->node_manager();
      node_manager_->Join(node_attr_, RoleType::ROLE_NODE);
    }
  }

  /**
   * @brief Destroy the Node Channel Impl object
   */
  virtual ~NodeChannelImpl() {
    if (is_reality_mode_) {
      node_manager_->Leave(node_attr_, RoleType::ROLE_NODE);
      node_manager_ = nullptr;
    }
  }

  /**
   * @brief get name of this node
   *
   * @return const std::string& actual node name
   */
  const std::string& NodeName() const { return node_name_; }

 private:
  template <typename MessageT>
  auto CreateWriter(const proto::RoleAttributes& role_attr)
      -> std::shared_ptr<Writer<MessageT>>;

  template <typename MessageT>
  auto CreateWriter(const std::string& channel_name)
      -> std::shared_ptr<Writer<MessageT>>;

  template <typename MessageT>
  auto CreateReader(const std::string& channel_name,
                    const CallbackFunc<MessageT>& reader_func)
      -> std::shared_ptr<Reader<MessageT>>;

  template <typename MessageT>
  auto CreateReader(const ReaderConfig& config,
                    const CallbackFunc<MessageT>& reader_func)
      -> std::shared_ptr<Reader<MessageT>>;

  template <typename MessageT>
  auto CreateReader(const proto::RoleAttributes& role_attr,
                    const CallbackFunc<MessageT>& reader_func,
                    uint32_t pending_queue_size = DEFAULT_PENDING_QUEUE_SIZE)
      -> std::shared_ptr<Reader<MessageT>>;

  template <typename MessageT>
  auto CreateReader(const proto::RoleAttributes& role_attr)
      -> std::shared_ptr<Reader<MessageT>>;

  template <typename MessageT>
  void FillInAttr(proto::RoleAttributes* attr);

  bool is_reality_mode_;
  std::string node_name_;
  proto::RoleAttributes node_attr_;
  NodeManagerPtr node_manager_ = nullptr;
};

template <typename MessageT>
auto NodeChannelImpl::CreateWriter(const proto::RoleAttributes& role_attr)
    -> std::shared_ptr<Writer<MessageT>> {
  if (!role_attr.has_channel_name() || role_attr.channel_name().empty()) {
    AERROR << "Can't create a writer with empty channel name!";
    return nullptr;
  }
  proto::RoleAttributes new_attr(role_attr);
  FillInAttr<MessageT>(&new_attr);

  std::shared_ptr<Writer<MessageT>> writer_ptr = nullptr;
  if (!is_reality_mode_) {
    writer_ptr = std::make_shared<blocker::IntraWriter<MessageT>>(new_attr);
  } else {
    writer_ptr = std::make_shared<Writer<MessageT>>(new_attr);
  }

  RETURN_VAL_IF_NULL(writer_ptr, nullptr);
  RETURN_VAL_IF(!writer_ptr->Init(), nullptr);
  return writer_ptr;
}

template <typename MessageT>
auto NodeChannelImpl::CreateWriter(const std::string& channel_name)
    -> std::shared_ptr<Writer<MessageT>> {
  proto::RoleAttributes role_attr;
  role_attr.set_channel_name(channel_name);
  return this->CreateWriter<MessageT>(role_attr);
}

template <typename MessageT>
auto NodeChannelImpl::CreateReader(const std::string& channel_name,
                                   const CallbackFunc<MessageT>& reader_func)
    -> std::shared_ptr<Reader<MessageT>> {
  proto::RoleAttributes role_attr;
  role_attr.set_channel_name(channel_name);
  return this->template CreateReader<MessageT>(role_attr, reader_func);
}

template <typename MessageT>
auto NodeChannelImpl::CreateReader(const ReaderConfig& config,
                                   const CallbackFunc<MessageT>& reader_func)
    -> std::shared_ptr<Reader<MessageT>> {
  proto::RoleAttributes role_attr;
  role_attr.set_channel_name(config.channel_name);
  role_attr.mutable_qos_profile()->CopyFrom(config.qos_profile);
  return this->template CreateReader<MessageT>(role_attr, reader_func,
                                               config.pending_queue_size);
}

template <typename MessageT>
auto NodeChannelImpl::CreateReader(const proto::RoleAttributes& role_attr,
                                   const CallbackFunc<MessageT>& reader_func,
                                   uint32_t pending_queue_size)
    -> std::shared_ptr<Reader<MessageT>> {
  if (!role_attr.has_channel_name() || role_attr.channel_name().empty()) {
    AERROR << "Can't create a reader with empty channel name!";
    return nullptr;
  }

  proto::RoleAttributes new_attr(role_attr);
  FillInAttr<MessageT>(&new_attr);

  std::shared_ptr<Reader<MessageT>> reader_ptr = nullptr;
  if (!is_reality_mode_) {
    reader_ptr =
        std::make_shared<blocker::IntraReader<MessageT>>(new_attr, reader_func);
  } else {
    reader_ptr = std::make_shared<Reader<MessageT>>(new_attr, reader_func,
                                                    pending_queue_size);
  }

  RETURN_VAL_IF_NULL(reader_ptr, nullptr);
  RETURN_VAL_IF(!reader_ptr->Init(), nullptr);
  return reader_ptr;
}

template <typename MessageT>
auto NodeChannelImpl::CreateReader(const proto::RoleAttributes& role_attr)
    -> std::shared_ptr<Reader<MessageT>> {
  return this->template CreateReader<MessageT>(role_attr, nullptr);
}

template <typename MessageT>
void NodeChannelImpl::FillInAttr(proto::RoleAttributes* attr) {
  attr->set_host_name(node_attr_.host_name());
  attr->set_host_ip(node_attr_.host_ip());
  attr->set_process_id(node_attr_.process_id());
  attr->set_node_name(node_attr_.node_name());
  attr->set_node_id(node_attr_.node_id());
  auto channel_id = GlobalData::RegisterChannel(attr->channel_name());
  attr->set_channel_id(channel_id);
  if (!attr->has_message_type()) {
    attr->set_message_type(message::MessageType<MessageT>());
  }
  if (!attr->has_proto_desc()) {
    std::string proto_desc("");
    message::GetDescriptorString<MessageT>(attr->message_type(), &proto_desc);
    attr->set_proto_desc(proto_desc);
  }
  if (!attr->has_qos_profile()) {
    attr->mutable_qos_profile()->CopyFrom(
        transport::QosProfileConf::QOS_PROFILE_DEFAULT);
  }
}

}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_NODE_NODE_CHANNEL_IMPL_H_
