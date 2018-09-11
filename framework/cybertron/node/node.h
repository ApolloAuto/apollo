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

#ifndef CYBERTRON_NODE_NODE_H_
#define CYBERTRON_NODE_NODE_H_

#include <memory>
#include <string>

#include "cybertron/node/node_channel_impl.h"
#include "cybertron/node/node_service_impl.h"

namespace apollo {
namespace cybertron {

template <typename M0, typename M1, typename M2>
class Component;
class TimerComponent;

class Node {
 public:
  template <typename M0, typename M1, typename M2>
  friend class Component;
  friend class TimerComponent;
  friend std::unique_ptr<Node> CreateNode(const std::string&,
                                          const std::string&);

  virtual ~Node();

  const std::string& Name() const;

  template <typename MessageT>
  auto CreateWriter(const proto::RoleAttributes& role_attr)
      -> std::shared_ptr<Writer<MessageT>>;

  template <typename MessageT>
  auto CreateReader(const proto::RoleAttributes& role_attr,
                    const CallbackFunc<MessageT>& reader_func)
      -> std::shared_ptr<cybertron::Reader<MessageT>>;

  template <typename MessageT>
  auto CreateWriter(const std::string& channel_name)
      -> std::shared_ptr<Writer<MessageT>>;

  template <typename MessageT>
  auto CreateReader(const std::string& channel_name,
                    const CallbackFunc<MessageT>& reader_func)
      -> std::shared_ptr<Reader<MessageT>>;

  template <typename Request, typename Response>
  auto CreateService(const std::string& service_name,
                     const typename Service<Request, Response>::ServiceCallback&
                         service_calllback)
      -> std::shared_ptr<Service<Request, Response>>;

  template <typename Request, typename Response>
  auto CreateClient(const std::string& service_name)
      -> std::shared_ptr<Client<Request, Response>>;

 private:
  explicit Node(const std::string& node_name,
                const std::string& name_space = "");
  template <typename MessageT>
  auto CreateReader(const proto::RoleAttributes& role_attr)
      -> std::shared_ptr<Reader<MessageT>>;

  std::unique_ptr<NodeChannelImpl> node_channel_impl_ = nullptr;
  std::unique_ptr<NodeServiceImpl> node_service_impl_ = nullptr;
  std::shared_ptr<topology::NodeManager> node_manager_ = nullptr;
  std::string node_name_;
  std::string name_space_;
  proto::RoleAttributes attr_;
};

template <typename MessageT>
auto Node::CreateWriter(const proto::RoleAttributes& role_attr)
    -> std::shared_ptr<Writer<MessageT>> {
  return node_channel_impl_->template CreateWriter<MessageT>(role_attr);
}

template <typename MessageT>
auto Node::CreateWriter(const std::string& channel_name)
    -> std::shared_ptr<Writer<MessageT>> {
  return node_channel_impl_->template CreateWriter<MessageT>(channel_name);
}

template <typename MessageT>
auto Node::CreateReader(const proto::RoleAttributes& role_attr)
    -> std::shared_ptr<Reader<MessageT>> {
  return node_channel_impl_->template CreateReader<MessageT>(role_attr);
}

template <typename MessageT>
auto Node::CreateReader(const proto::RoleAttributes& role_attr,
                        const CallbackFunc<MessageT>& reader_func)
    -> std::shared_ptr<Reader<MessageT>> {
  return node_channel_impl_->template CreateReader<MessageT>(role_attr,
                                                             reader_func);
}

template <typename MessageT>
auto Node::CreateReader(const std::string& channel_name,
                        const CallbackFunc<MessageT>& reader_func)
    -> std::shared_ptr<Reader<MessageT>> {
  return node_channel_impl_->template CreateReader<MessageT>(channel_name,
                                                             reader_func);
}

template <typename Request, typename Response>
auto Node::CreateService(
    const std::string& service_name,
    const typename Service<Request, Response>::ServiceCallback&
        service_calllback) -> std::shared_ptr<Service<Request, Response>> {
  return node_service_impl_->template CreateService<Request, Response>(
      service_name, service_calllback);
}

template <typename Request, typename Response>
auto Node::CreateClient(const std::string& service_name)
    -> std::shared_ptr<Client<Request, Response>> {
  return node_service_impl_->template CreateClient<Request, Response>(
      service_name);
}

}  // namespace cybertron
}  // namespace apollo

#endif  // CYBERTRON_NODE_NODE_H_
