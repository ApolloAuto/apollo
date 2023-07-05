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

#include "cyber/node/node.h"

#include "cyber/common/global_data.h"
#include "cyber/time/time.h"

namespace apollo {
namespace cyber {

using proto::RoleType;

Node::Node(const std::string& node_name, const std::string& name_space)
    : node_name_(node_name), name_space_(name_space) {
  node_channel_impl_.reset(new NodeChannelImpl(node_name));
  node_service_impl_.reset(new NodeServiceImpl(node_name));
}

Node::~Node() {}

const std::string& Node::Name() const { return node_name_; }

void Node::Observe() {
  for (auto& reader : readers_) {
    reader.second->Observe();
  }
}

void Node::ClearData() {
  for (auto& reader : readers_) {
    reader.second->ClearData();
  }
}

bool Node::DeleteReader(const std::string& channel_name) {
  std::lock_guard<std::mutex> lg(readers_mutex_);
  int result = readers_.erase(channel_name);
  if (1 == result) return true;
  return false;
}

bool Node::DeleteReader(const proto::RoleAttributes& role_attr) {
  std::lock_guard<std::mutex> lg(readers_mutex_);
  int result = readers_.erase(role_attr.channel_name());
  if (1 == result) return true;
  return false;
}

bool Node::DeleteReader(const ReaderConfig& config) {
  std::lock_guard<std::mutex> lg(readers_mutex_);
  int result = readers_.erase(config.channel_name);
  if (1 == result) return true;
  return false;
}
}  // namespace cyber
}  // namespace apollo
