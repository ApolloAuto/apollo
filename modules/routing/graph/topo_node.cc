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

#include "modules/routing/graph/topo_node.h"

#include <math.h>

#include "modules/routing/graph/topo_edge.h"

namespace apollo {
namespace routing {

namespace {

using ::apollo::routing::common::Node;
using ::apollo::routing::common::Edge;

}  // namespace

TopoNode::TopoNode(const Node& node)
    : _pb_node(node), _start_s(0.0), _end_s(_pb_node.length()) {
  int total_size = 0;
  for (const auto& seg : central_curve().segment()) {
    total_size += seg.line_segment().point_size();
  }
  int half_size = total_size / 2;
  for (const auto& seg : central_curve().segment()) {
    if (half_size < seg.line_segment().point_size()) {
      _anchor_point = seg.line_segment().point(half_size);
      break;
    }
    half_size -= seg.line_segment().point_size();
  }
  _origin_node = this;
}

TopoNode::TopoNode(const TopoNode* topo_node) : TopoNode(topo_node->node()) {}

TopoNode::~TopoNode() {}

const Node& TopoNode::node() const { return _pb_node; }

double TopoNode::length() const { return _pb_node.length(); }

double TopoNode::cost() const { return _pb_node.cost(); }

bool TopoNode::is_virtual() const { return _pb_node.is_virtual(); }

const std::string& TopoNode::lane_id() const { return _pb_node.lane_id(); }

const std::string& TopoNode::road_id() const { return _pb_node.road_id(); }

const ::apollo::common::hdmap::Curve& TopoNode::central_curve() const {
  return _pb_node.central_curve();
}

const ::apollo::common::hdmap::Point& TopoNode::anchor_point() const {
  return _anchor_point;
}

const std::unordered_set<const TopoEdge*>& TopoNode::in_from_all_edge() const {
  return _in_from_all_edge_set;
}

const std::unordered_set<const TopoEdge*>& TopoNode::in_from_left_edge() const {
  return _in_from_left_edge_set;
}

const std::unordered_set<const TopoEdge*>& TopoNode::in_from_right_edge()
    const {
  return _in_from_right_edge_set;
}

const std::unordered_set<const TopoEdge*>&
TopoNode::in_from_left_or_right_edge() const {
  return _in_from_left_or_right_edge_set;
}

const std::unordered_set<const TopoEdge*>& TopoNode::in_from_pre_edge() const {
  return _in_from_pre_edge_set;
}

const std::unordered_set<const TopoEdge*>& TopoNode::out_to_all_edge() const {
  return _out_to_all_edge_set;
}

const std::unordered_set<const TopoEdge*>& TopoNode::out_to_left_edge() const {
  return _out_to_left_edge_set;
}

const std::unordered_set<const TopoEdge*>& TopoNode::out_to_right_edge() const {
  return _out_to_right_edge_set;
}

const std::unordered_set<const TopoEdge*>& TopoNode::out_to_left_or_right_edge()
    const {
  return _out_to_left_or_right_edge_set;
}

const std::unordered_set<const TopoEdge*>& TopoNode::out_to_suc_edge() const {
  return _out_to_suc_edge_set;
}

const TopoEdge* TopoNode::get_in_edge_from(const TopoNode* from_node) const {
  const auto& iter = _in_edge_map.find(from_node);
  if (iter == _in_edge_map.end()) {
    return nullptr;
  }
  return iter->second;
}

const TopoEdge* TopoNode::get_out_edge_to(const TopoNode* to_node) const {
  const auto& iter = _out_edge_map.find(to_node);
  if (iter == _out_edge_map.end()) {
    return nullptr;
  }
  return iter->second;
}

const TopoNode* TopoNode::origin_node() const { return _origin_node; }

double TopoNode::start_s() const { return _start_s; }

double TopoNode::end_s() const { return _end_s; }

bool TopoNode::is_sub_node() const { return origin_node() != this; }

void TopoNode::add_in_edge(const TopoEdge* edge) {
  if (edge->to_node() != this) {
    return;
  }
  if (_in_edge_map.count(edge->from_node()) != 0) {
    return;
  }
  switch (edge->type()) {
    case TET_LEFT:
      _in_from_right_edge_set.insert(edge);
      _in_from_left_or_right_edge_set.insert(edge);
      break;
    case TET_RIGHT:
      _in_from_left_edge_set.insert(edge);
      _in_from_left_or_right_edge_set.insert(edge);
      break;
    default:
      _in_from_pre_edge_set.insert(edge);
      break;
  }
  _in_from_all_edge_set.insert(edge);
  _in_edge_map[edge->from_node()] = edge;
}

void TopoNode::add_out_edge(const TopoEdge* edge) {
  if (edge->from_node() != this) {
    return;
  }
  if (_out_edge_map.count(edge->to_node()) != 0) {
    return;
  }
  switch (edge->type()) {
    case TET_LEFT:
      _out_to_left_edge_set.insert(edge);
      _out_to_left_or_right_edge_set.insert(edge);
      break;
    case TET_RIGHT:
      _out_to_right_edge_set.insert(edge);
      _out_to_left_or_right_edge_set.insert(edge);
      break;
    default:
      _out_to_suc_edge_set.insert(edge);
      break;
  }
  _out_to_all_edge_set.insert(edge);
  _out_edge_map[edge->to_node()] = edge;
}

void TopoNode::set_origin_node(const TopoNode* origin_node) {
  _origin_node = origin_node;
}

void TopoNode::set_start_s(double start_s) { _start_s = start_s; }

void TopoNode::set_end_s(double end_s) { _end_s = end_s; }

}  // namespace routing
}  // namespace apollo
