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

/**
 * @file logic_node.cc
 **/

#include "modules/planning/common/logic_node.h"

#include "modules/common/log.h"

namespace apollo {
namespace planning {

LogicNode::LogicNode(const std::uint32_t node_id, const std::string &lane_id)
    : _node_id(node_id), _lane_id(lane_id) {}

std::uint32_t LogicNode::node_id() const { return _node_id; }

const std::string &LogicNode::lane_id() const { return _lane_id; }

const std::unordered_set<std::string> &LogicNode::lane_name() const {
  return _lane_name;
}

void LogicNode::add_lane_name(const std::string &lane_name) {
  _lane_name.insert(lane_name);
}

void LogicNode::connect(const LogicNode &node) {
  CHECK(_edge.find(node.lane_id()) == _edge.end());
  _edge[node.lane_id()] = node.node_id();
}

common::ErrorCode LogicNode::get_next_node(const std::string &lane_id,
                                           std::uint32_t *const node_id) {
  CHECK_NOTNULL(node_id);
  *node_id = 0;
  if (_edge.find(lane_id) == _edge.end()) {
    return common::ErrorCode::PLANNING_ERROR;
  }
  *node_id = _edge[lane_id];
  return common::ErrorCode::OK;
}

}  // namespace planning
}  // namespace apollo
