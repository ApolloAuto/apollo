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

#include "graph/topo_node.h"

#include <math.h>

#include "graph/topo_edge.h"

#include "glog/logging.h"

namespace adu {
namespace routing {

namespace {

using ::google::protobuf::RepeatedPtrField;

using ::adu::routing::common::Node;
using ::adu::routing::common::Edge;
using ::adu::routing::common::CurveRange;
using ::adu::common::hdmap::Curve;

bool find_anchor_point(const Curve& curve, ::adu::common::hdmap::Point* anchor_point) {
    int total_size = 0;
    for (const auto& seg : curve.segment()) {
        total_size += seg.line_segment().point_size();
    }
    int half_size = total_size / 2;
    for (const auto& seg : curve.segment()) {
        if (half_size < seg.line_segment().point_size()) {
            *anchor_point = seg.line_segment().point(half_size);
            return true;
        }
        half_size -= seg.line_segment().point_size();
    }
    return false;
}

void convert_out_range(const RepeatedPtrField<CurveRange>& range_vec,
                       double start_s,
                       double end_s,
                       std::vector<NodeSRange>* out_range,
                       int* prefer_index) {
    double s_s = 0.0;
    double e_s = 0.0;
    for (const auto& c_range : range_vec) {
        s_s = c_range.start().s();
        e_s = c_range.end().s();
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
        if (out_range->at(i).length() > max_diff) {
            max_index = i;
            max_diff = out_range->at(i).length();
        }
    }
    *prefer_index = max_index;
}

} // namespace

bool TopoNode::is_out_range_enough(const std::vector<NodeSRange>& range_vec,
                                   double start_s,
                                   double end_s) {
    int start_index = binary_search_for_s_larger(range_vec, start_s);
    int end_index = binary_search_for_s_smaller(range_vec, end_s);

    int index_diff = end_index - start_index;
    if (start_index < 0 || end_index < 0) {
        return false;
    }
    if (index_diff > 1) {
        return true;
    }

    double pre_s_s = std::max(start_s, range_vec[start_index].start_s());
    double suc_e_s = std::min(end_s, range_vec[end_index].end_s());

    if (index_diff == 1) {
        double dlt = range_vec[start_index].end_s() - pre_s_s;
        dlt += suc_e_s - range_vec[end_index].start_s();
        return NodeSRange::is_enough_for_change_lane(dlt);
    }
    if (index_diff == 0) {
        return NodeSRange::is_enough_for_change_lane(pre_s_s, suc_e_s);
    }
    return false;
}

TopoNode::TopoNode(const Node& node) : _pb_node(node), _start_s(0.0), _end_s(_pb_node.length()) {
    init();
    _origin_node = this;
}

TopoNode::TopoNode(const TopoNode* topo_node, const NodeSRange& range) :
        _pb_node(topo_node->node()) {
    _origin_node = topo_node;
    _start_s = range.start_s();
    _end_s = range.end_s();
    init();
}

TopoNode::~TopoNode() { }

void TopoNode::init() {
    if (!find_anchor_point(central_curve(), &_anchor_point)) {
        LOG(WARNING) << "Be attention!!! Find anchor point failed for lane: " << lane_id();
    }
    convert_out_range(_pb_node.left_out(), _start_s, _end_s,
                      &_left_out_sorted_range, &_left_prefer_range_index);
    _is_left_range_enough = (_left_prefer_range_index >= 0) &&
            _left_out_sorted_range[_left_prefer_range_index].is_enough_for_change_lane();

    convert_out_range(_pb_node.right_out(), _start_s, _end_s,
                      &_right_out_sorted_range, &_right_prefer_range_index);
    _is_right_range_enough = (_right_prefer_range_index >= 0) &&
            _right_out_sorted_range[_right_prefer_range_index].is_enough_for_change_lane();
}

const Node& TopoNode::node() const {
    return _pb_node;
}

double TopoNode::length() const {
    return _pb_node.length();
}

double TopoNode::cost() const {
    return _pb_node.cost();
}

bool TopoNode::is_virtual() const {
    return _pb_node.is_virtual();
}

const std::string& TopoNode::lane_id() const {
    return _pb_node.lane_id();
}

const std::string& TopoNode::road_id() const {
    return _pb_node.road_id();
}

const ::adu::common::hdmap::Curve& TopoNode::central_curve() const {
    return _pb_node.central_curve();
}

const ::adu::common::hdmap::Point& TopoNode::anchor_point() const {
    return _anchor_point;
}

const std::vector<NodeSRange>& TopoNode::left_out_range() const {
    return _left_out_sorted_range;
}

const std::vector<NodeSRange>& TopoNode::right_out_range() const {
    return _right_out_sorted_range;
}

const std::unordered_set<const TopoEdge*>& TopoNode::in_from_all_edge() const {
    return _in_from_all_edge_set;
}

const std::unordered_set<const TopoEdge*>& TopoNode::in_from_left_edge() const {
    return _in_from_left_edge_set;
}

const std::unordered_set<const TopoEdge*>& TopoNode::in_from_right_edge() const {
    return _in_from_right_edge_set;
}

const std::unordered_set<const TopoEdge*>& TopoNode::in_from_left_or_right_edge() const {
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

const std::unordered_set<const TopoEdge*>& TopoNode::out_to_left_or_right_edge() const {
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

const TopoNode* TopoNode::origin_node() const {
    return _origin_node;
}

double TopoNode::start_s() const {
    return _start_s;
}

double TopoNode::end_s() const {
    return _end_s;
}

bool TopoNode::is_sub_node() const {
    return origin_node() != this;
}

bool TopoNode::is_overlap_enough(const TopoNode* sub_node, const TopoEdge* edge_for_type) const {
    if (edge_for_type->type() == TET_LEFT) {
        return (_is_left_range_enough &&
            is_out_range_enough(_left_out_sorted_range, sub_node->start_s(), sub_node->end_s()));
    }
    if (edge_for_type->type() == TET_RIGHT) {
        return (_is_right_range_enough &&
            is_out_range_enough(_right_out_sorted_range, sub_node->start_s(), sub_node->end_s()));
    }
    return true;
}

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

} // namespace routing
} // namespace adu

