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

#include <algorithm>
#include <cmath>
#include <utility>

#include "cyber/common/log.h"
#include "modules/common/util/map_util.h"
#include "modules/routing/graph/range_utils.h"

namespace apollo {
namespace routing {

namespace {

const double MIN_INTERNAL_FOR_NODE = 0.01;  // in meter
const double kLenghtEpsilon = 1e-6;         // in meter

using apollo::common::util::FindPtrOrNull;
using ::google::protobuf::RepeatedPtrField;

void ConvertOutRange(const RepeatedPtrField<CurveRange>& range_vec,
                     double start_s, double end_s,
                     std::vector<NodeSRange>* out_range, int* prefer_index) {
  out_range->clear();
  for (const auto& c_range : range_vec) {
    double s_s = c_range.start().s();
    double e_s = c_range.end().s();
    if (e_s < start_s || s_s > end_s || e_s < s_s) {
      continue;
    }
    s_s = std::max(start_s, s_s);
    e_s = std::min(end_s, e_s);
    NodeSRange s_range(s_s, e_s);
    out_range->push_back(std::move(s_range));
  }
  sort(out_range->begin(), out_range->end());
  int max_index = -1;
  double max_diff = 0.0;
  for (size_t i = 0; i < out_range->size(); ++i) {
    if (out_range->at(i).Length() > max_diff) {
      max_index = static_cast<int>(i);
      max_diff = out_range->at(i).Length();
    }
  }
  *prefer_index = max_index;
}

}  // namespace

bool TopoNode::IsOutRangeEnough(const std::vector<NodeSRange>& range_vec,
                                double start_s, double end_s) {
  if (!NodeSRange::IsEnoughForChangeLane(start_s, end_s)) {
    return false;
  }
  int start_index = BinarySearchForSLarger(range_vec, start_s);
  int end_index = BinarySearchForSSmaller(range_vec, end_s);

  int index_diff = end_index - start_index;
  if (start_index < 0 || end_index < 0) {
    return false;
  }
  if (index_diff > 1) {
    return true;
  }

  double pre_s_s = std::max(start_s, range_vec[start_index].StartS());
  double suc_e_s = std::min(end_s, range_vec[end_index].EndS());

  if (index_diff == 1) {
    double dlt = range_vec[start_index].EndS() - pre_s_s;
    dlt += suc_e_s - range_vec[end_index].StartS();
    return NodeSRange::IsEnoughForChangeLane(dlt);
  }
  if (index_diff == 0) {
    return NodeSRange::IsEnoughForChangeLane(pre_s_s, suc_e_s);
  }
  return false;
}

TopoNode::TopoNode(const Node& node)
    : pb_node_(node), start_s_(0.0), end_s_(pb_node_.length()) {
  ACHECK(pb_node_.length() > kLenghtEpsilon)
      << "Node length is invalid in pb: " << pb_node_.DebugString();
  Init();
  origin_node_ = this;
}

TopoNode::TopoNode(const TopoNode* topo_node, const NodeSRange& range)
    : TopoNode(topo_node->PbNode()) {
  origin_node_ = topo_node;
  start_s_ = range.StartS();
  end_s_ = range.EndS();
  Init();
}

TopoNode::~TopoNode() {}

void TopoNode::Init() {
  if (!FindAnchorPoint()) {
    AWARN << "Be attention!!! Find anchor point failed for lane: " << LaneId();
  }
  ConvertOutRange(pb_node_.left_out(), start_s_, end_s_,
                  &left_out_sorted_range_, &left_prefer_range_index_);

  is_left_range_enough_ =
      (left_prefer_range_index_ >= 0) &&
      left_out_sorted_range_[left_prefer_range_index_].IsEnoughForChangeLane();

  ConvertOutRange(pb_node_.right_out(), start_s_, end_s_,
                  &right_out_sorted_range_, &right_prefer_range_index_);
  is_right_range_enough_ = (right_prefer_range_index_ >= 0) &&
                           right_out_sorted_range_[right_prefer_range_index_]
                               .IsEnoughForChangeLane();
}

bool TopoNode::FindAnchorPoint() {
  double total_size = 0;
  for (const auto& seg : CentralCurve().segment()) {
    total_size += seg.line_segment().point_size();
  }
  double rate = (StartS() + EndS()) / 2.0 / Length();
  int anchor_index = static_cast<int>(total_size * rate);
  for (const auto& seg : CentralCurve().segment()) {
    if (anchor_index < seg.line_segment().point_size()) {
      SetAnchorPoint(seg.line_segment().point(anchor_index));
      return true;
    }
    anchor_index -= seg.line_segment().point_size();
  }
  return false;
}

void TopoNode::SetAnchorPoint(const common::PointENU& anchor_point) {
  anchor_point_ = anchor_point;
}

const Node& TopoNode::PbNode() const { return pb_node_; }

double TopoNode::Length() const { return pb_node_.length(); }

double TopoNode::Cost() const { return pb_node_.cost(); }

bool TopoNode::IsVirtual() const { return pb_node_.is_virtual(); }

const std::string& TopoNode::LaneId() const { return pb_node_.lane_id(); }

const std::string& TopoNode::RoadId() const { return pb_node_.road_id(); }

const hdmap::Curve& TopoNode::CentralCurve() const {
  return pb_node_.central_curve();
}

const common::PointENU& TopoNode::AnchorPoint() const { return anchor_point_; }

const std::vector<NodeSRange>& TopoNode::LeftOutRange() const {
  return left_out_sorted_range_;
}

const std::vector<NodeSRange>& TopoNode::RightOutRange() const {
  return right_out_sorted_range_;
}

const std::unordered_set<const TopoEdge*>& TopoNode::InFromAllEdge() const {
  return in_from_all_edge_set_;
}

const std::unordered_set<const TopoEdge*>& TopoNode::InFromLeftEdge() const {
  return in_from_left_edge_set_;
}

const std::unordered_set<const TopoEdge*>& TopoNode::InFromRightEdge() const {
  return in_from_right_edge_set_;
}

const std::unordered_set<const TopoEdge*>& TopoNode::InFromLeftOrRightEdge()
    const {
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
  return FindPtrOrNull(in_edge_map_, from_node);
}

const TopoEdge* TopoNode::GetOutEdgeTo(const TopoNode* to_node) const {
  return FindPtrOrNull(out_edge_map_, to_node);
}

const TopoNode* TopoNode::OriginNode() const { return origin_node_; }

double TopoNode::StartS() const { return start_s_; }

double TopoNode::EndS() const { return end_s_; }

bool TopoNode::IsSubNode() const { return OriginNode() != this; }

bool TopoNode::IsOverlapEnough(const TopoNode* sub_node,
                               const TopoEdge* edge_for_type) const {
  if (edge_for_type->Type() == TET_LEFT) {
    return (is_left_range_enough_ &&
            IsOutRangeEnough(left_out_sorted_range_, sub_node->StartS(),
                             sub_node->EndS()));
  }
  if (edge_for_type->Type() == TET_RIGHT) {
    return (is_right_range_enough_ &&
            IsOutRangeEnough(right_out_sorted_range_, sub_node->StartS(),
                             sub_node->EndS()));
  }
  if (edge_for_type->Type() == TET_FORWARD) {
    return IsOutToSucEdgeValid() && sub_node->IsInFromPreEdgeValid();
  }
  return true;
}

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

bool TopoNode::IsInFromPreEdgeValid() const {
  return std::fabs(StartS() - OriginNode()->StartS()) < MIN_INTERNAL_FOR_NODE;
}

bool TopoNode::IsOutToSucEdgeValid() const {
  return std::fabs(EndS() - OriginNode()->EndS()) < MIN_INTERNAL_FOR_NODE;
}

TopoEdge::TopoEdge(const Edge& edge, const TopoNode* from_node,
                   const TopoNode* to_node)
    : pb_edge_(edge), from_node_(from_node), to_node_(to_node) {}

TopoEdge::~TopoEdge() {}

const Edge& TopoEdge::PbEdge() const { return pb_edge_; }

double TopoEdge::Cost() const { return pb_edge_.cost(); }

const TopoNode* TopoEdge::FromNode() const { return from_node_; }

const TopoNode* TopoEdge::ToNode() const { return to_node_; }

const std::string& TopoEdge::FromLaneId() const {
  return pb_edge_.from_lane_id();
}

const std::string& TopoEdge::ToLaneId() const { return pb_edge_.to_lane_id(); }

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
