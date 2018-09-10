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

#include "cybertron/node/node.h"
#include "cybertron/common/global_data.h"
#include "cybertron/time/time.h"

namespace apollo {
namespace cybertron {

using proto::RoleType;

Node::Node(const std::string& node_name, const std::string& name_space)
    : node_name_(node_name), name_space_(name_space) {
  node_channel_impl_.reset(new NodeChannelImpl(node_name));
  node_service_impl_.reset(new NodeServiceImpl(node_name));

  attr_.set_host_name(common::GlobalData::Instance()->HostName());
  attr_.set_process_id(common::GlobalData::Instance()->ProcessId());
  attr_.set_node_name(node_name);
  uint64_t node_id = common::GlobalData::RegisterNode(node_name);
  attr_.set_node_id(node_id);

  node_manager_ = topology::Topology::Instance()->node_manager();
  std::weak_ptr<topology::NodeManager> wk_mgr = node_manager_;
  if (auto mgr = wk_mgr.lock()) {
    mgr->Join(attr_, RoleType::ROLE_NODE);
  }
}

Node::~Node() {
  std::weak_ptr<topology::NodeManager> wk_mgr = node_manager_;
  if (auto mgr = wk_mgr.lock()) {
    mgr->Leave(attr_, RoleType::ROLE_NODE);
  }
}

const std::string& Node::Name() const { return node_name_; }

}  // namespace cybertron
}  // namespace apollo
