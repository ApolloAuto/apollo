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

#ifndef CYBERTRON_NODE_NODE_CHANNEL_IMPL_H_
#define CYBERTRON_NODE_NODE_CHANNEL_IMPL_H_

#include <memory>
#include <string>

#include "cybertron/message/message_traits.h"
#include "cybertron/node/reader.h"
#include "cybertron/node/writer.h"

namespace apollo {
namespace cybertron {

class Node;
class NodeChannelImpl {
 public:
  friend class Node;

  explicit NodeChannelImpl(const std::string& node_name)
      : node_name_(node_name) {
    // TODO: topology register node
  }
  virtual ~NodeChannelImpl() {}

 private:
  std::string NodeName() const { return node_name_; }

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
  auto CreateReader(const proto::RoleAttributes& role_attr,
                    const CallbackFunc<MessageT>& reader_func)
      -> std::shared_ptr<Reader<MessageT>>;

  template <typename MessageT>
  auto CreateReader(const proto::RoleAttributes& role_attr)
      -> std::shared_ptr<Reader<MessageT>>;

  std::string node_name_;
};

template <typename MessageT>
auto NodeChannelImpl::CreateWriter(const proto::RoleAttributes& role_attr)
    -> std::shared_ptr<Writer<MessageT>> {
  auto channel_id = GlobalData::RegisterChannel(role_attr.channel_name());
  auto node_id = GlobalData::RegisterNode(node_name_);
  proto::RoleAttributes new_attr(role_attr);
  new_attr.set_node_name(node_name_);
  new_attr.set_node_id(node_id);
  new_attr.set_channel_id(channel_id);
  if (!new_attr.has_message_type()) {
    new_attr.set_message_type(message::MessageType<MessageT>());
  }
  if (!new_attr.has_proto_desc()) {
    std::string proto_desc("");
    message::GetDescriptorString<MessageT>(new_attr.message_type(),
                                           &proto_desc);
    new_attr.set_proto_desc(proto_desc);
  }
  std::shared_ptr<Writer<MessageT>> writer_ptr =
      std::make_shared<Writer<MessageT>>(new_attr);
  RETURN_VAL_IF_NULL(writer_ptr, nullptr);
  RETURN_VAL_IF(!writer_ptr->Init(), nullptr);
  return writer_ptr;
}

template <typename MessageT>
auto NodeChannelImpl::CreateWriter(const std::string& channel_name)
    -> std::shared_ptr<Writer<MessageT>> {
  proto::RoleAttributes role_attr;
  role_attr.set_channel_name(channel_name);
  return this->template CreateWriter<MessageT>(role_attr);
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
auto NodeChannelImpl::CreateReader(const proto::RoleAttributes& role_attr,
                                   const CallbackFunc<MessageT>& reader_func)
    -> std::shared_ptr<Reader<MessageT>> {
  auto channel_id = GlobalData::RegisterChannel(role_attr.channel_name());
  auto node_id = GlobalData::RegisterNode(node_name_);
  proto::RoleAttributes new_attr(role_attr);
  new_attr.set_node_name(node_name_);
  new_attr.set_node_id(node_id);
  new_attr.set_channel_id(channel_id);
  if (!new_attr.has_message_type()) {
    new_attr.set_message_type(message::MessageType<MessageT>());
  }
  std::shared_ptr<Reader<MessageT>> reader_ptr =
      std::make_shared<Reader<MessageT>>(new_attr, reader_func);
  RETURN_VAL_IF_NULL(reader_ptr, nullptr);
  RETURN_VAL_IF(!reader_ptr->Init(), nullptr);
  return reader_ptr;
}

template <typename MessageT>
auto NodeChannelImpl::CreateReader(const proto::RoleAttributes& role_attr)
    -> std::shared_ptr<Reader<MessageT>> {
  auto channel_id = GlobalData::RegisterChannel(role_attr.channel_name());
  auto node_id = GlobalData::RegisterNode(node_name_);
  proto::RoleAttributes new_attr(role_attr);
  new_attr.set_node_name(node_name_);
  new_attr.set_node_id(node_id);
  new_attr.set_channel_id(channel_id);
  if (!new_attr.has_message_type()) {
    new_attr.set_message_type(message::MessageType<MessageT>());
  }
  std::shared_ptr<Reader<MessageT>> reader_ptr =
      std::make_shared<Reader<MessageT>>(new_attr, nullptr);
  RETURN_VAL_IF_NULL(reader_ptr, nullptr);
  RETURN_VAL_IF(!reader_ptr->Init(), nullptr);
  return reader_ptr;
}

}  // namespace cybertron
}  // namespace apollo

#endif  // CYBERTRON_NODE_NODE_CHANNEL_IMPL_H_
