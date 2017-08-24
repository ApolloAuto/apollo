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

namespace apollo {
namespace routing {

TopoNode::TopoNode(const Node& node)
    : pb_node_(node), start_s_(0.0), end_s_(pb_node_.length()) {
  int total_size = 0;
  for (const auto& seg : CentralCurve().segment()) {
    total_size += seg.line_segment().point_size();
  }
  int half_size = total_size / 2;
  for (const auto& seg : CentralCurve().segment()) {
    if (half_size < seg.line_segment().point_size()) {
      anchor_point_ = seg.line_segment().point(half_size);
      break;
    }
    half_size -= seg.line_segment().point_size();
  }
  origin_node_ = this;
}

TopoNode::TopoNode(const TopoNode* topo_node) : TopoNode(topo_node->PbNode()) {}

TopoNode::~TopoNode() {}

const Node& TopoNode::PbNode() const { return pb_node_; }

double TopoNode::Length() const { return pb_node_.length(); }

double TopoNode::Cost() const { return pb_node_.cost(); }

bool TopoNode::IsVirtual() const { return pb_node_.is_virtual(); }

const std::string& TopoNode::LaneId() const { return pb_node_.lane_id(); }

const std::string& TopoNode::RoadId() const { return pb_node_.road_id(); }

const ::apollo::hdmap::Curve& TopoNode::CentralCurve() const {
  return pb_node_.central_curve();
}

const ::apollo::common::PointENU& TopoNode::AnchorPoint() const {
  return anchor_point_;
}

const std::unordered_set<const TopoEdge*>& TopoNode::InFromAllEdge() const {
  return in_from_all_edge_set_;
}

const std::unordered_set<const TopoEdge*>& TopoNode::InFromLeftEdge() const {
  return in_from_left_edge_set_;
}

const std::unordered_set<const TopoEdge*>& TopoNode::InFromRightEdge()
    const {
  return in_from_right_edge_set_;
}

const std::unordered_set<const TopoEdge*>&
TopoNode::InFromLeftOrRightEdge() const {
  return in_from_left_or_right_edge_set_;
}

const std::unordered_set<const TopoEdge*>& TopoNode::InFromPreEdge() const {
  return in_from_pre_edge_set_;
}

const std::unordered_set<const TopoEdge*>& TopoNode::OutToAllEdge() const {
  return out_to_all_edge_set_;
}

const std::unordered_set<const TopoEdge*>& TopoNode::OutToLeftEdge() const {
  return out_to_left_edge_set_;
}

const std::unordered_set<const TopoEdge*>& TopoNode::OutToRightEdge() const {
  return out_to_right_edge_set_;
}

const std::unordered_set<const TopoEdge*>& TopoNode::OutToLeftOrRightEdge()
    const {
  return out_to_left_or_right_edge_set_;
}

const std::unordered_set<const TopoEdge*>& TopoNode::OutToSucEdge() const {
  return out_to_suc_edge_set_;
}

const TopoEdge* TopoNode::GetInEdgeFrom(const TopoNode* from_node) const {
  const auto& iter = in_edge_map_.find(from_node);
  if (iter == in_edge_map_.end()) {
    return nullptr;
  }
  return iter->second;
}

const TopoEdge* TopoNode::GetOutEdgeTo(const TopoNode* to_node) const {
  const auto& iter = out_edge_map_.find(to_node);
  if (iter == out_edge_map_.end()) {
    return nullptr;
  }
  return iter->second;
}

const TopoNode* TopoNode::OriginNode() const { return origin_node_; }

double TopoNode::StartS() const { return start_s_; }

double TopoNode::EndS() const { return end_s_; }

bool TopoNode::IsSubNode() const { return OriginNode() != this; }

void TopoNode::AddInEdge(const TopoEdge* edge) {
  if (edge->ToNode() != this) {
    return;
  }
  if (in_edge_map_.count(edge->FromNode()) != 0) {
    return;
  }
  switch (edge->Type()) {
    case TET_LEFT:
      in_from_right_edge_set_.insert(edge);
      in_from_left_or_right_edge_set_.insert(edge);
      break;
    case TET_RIGHT:
      in_from_left_edge_set_.insert(edge);
      in_from_left_or_right_edge_set_.insert(edge);
      break;
    default:
      in_from_pre_edge_set_.insert(edge);
      break;
  }
  in_from_all_edge_set_.insert(edge);
  in_edge_map_[edge->FromNode()] = edge;
}

void TopoNode::AddOutEdge(const TopoEdge* edge) {
  if (edge->FromNode() != this) {
    return;
  }
  if (out_edge_map_.count(edge->ToNode()) != 0) {
    return;
  }
  switch (edge->Type()) {
    case TET_LEFT:
      out_to_left_edge_set_.insert(edge);
      out_to_left_or_right_edge_set_.insert(edge);
      break;
    case TET_RIGHT:
      out_to_right_edge_set_.insert(edge);
      out_to_left_or_right_edge_set_.insert(edge);
      break;
    default:
      out_to_suc_edge_set_.insert(edge);
      break;
  }
  out_to_all_edge_set_.insert(edge);
  out_edge_map_[edge->ToNode()] = edge;
}

void TopoNode::SetOriginNode(const TopoNode* origin_node) {
  origin_node_ = origin_node;
}

void TopoNode::SetStartS(double start_s) { start_s_ = start_s; }

void TopoNode::SetEndS(double end_s) { end_s_ = end_s; }

TopoEdge::TopoEdge(const Edge& edge,
                   const TopoNode* from_node, const TopoNode* to_node)
    : pb_edge_(edge), from_node_(from_node), to_node_(to_node) {}

TopoEdge::~TopoEdge() {}

const Edge& TopoEdge::PbEdge() const { return pb_edge_; }

double TopoEdge::Cost() const { return pb_edge_.cost(); }

const TopoNode* TopoEdge::FromNode() const { return from_node_; }

const TopoNode* TopoEdge::ToNode() const { return to_node_; }

const std::string& TopoEdge::FromLaneId() const {
  return pb_edge_.from_lane_id();
}

const std::string& TopoEdge::ToLaneId() const {
  return pb_edge_.to_lane_id();
}

TopoEdgeType TopoEdge::Type() const {
  if (pb_edge_.direction_type() == Edge::LEFT) {
    return TET_LEFT;
  }
  if (pb_edge_.direction_type() == Edge::RIGHT) {
    return TET_RIGHT;
  }
  return TET_FORWARD;
}
}  // namespace routing
}  // namespace apollo
