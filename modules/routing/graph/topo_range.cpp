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

#include "graph/topo_range.h"
#include "graph/topo_node.h"

namespace adu {
namespace routing {

class TopoNode;

const double MIN_LENGTH_FOR_CHANGE_LANE = 10.0;

bool NodeSRange::is_enough_for_change_lane(double start_s, double end_s) {
    return is_enough_for_change_lane(end_s - start_s);
}

bool NodeSRange::is_enough_for_change_lane(double length) {
    return (length > MIN_LENGTH_FOR_CHANGE_LANE);
}

NodeSRange::NodeSRange() : _start_s(0.0), _end_s(0.0) { }

NodeSRange::NodeSRange(double s1, double s2) : _start_s(s1), _end_s(s2) { }

NodeSRange::NodeSRange(const NodeSRange& other) : _start_s(other.start_s()), _end_s(other.end_s()) {
}

bool NodeSRange::operator < (const NodeSRange& other) const {
    return start_s() < other.start_s();
}

bool NodeSRange::is_valid() const {
    return _start_s <= _end_s;
}

double NodeSRange::start_s() const {
    return _start_s;
}

double NodeSRange::end_s() const {
    return _end_s;
}

double NodeSRange::length() const {
    return _end_s - _start_s;
}

bool NodeSRange::is_enough_for_change_lane() const {
    return NodeSRange::is_enough_for_change_lane(start_s(), end_s());
}

void NodeSRange::set_start_s(double start_s) {
    _start_s = start_s;
}

void NodeSRange::set_end_s(double end_s) {
    _end_s = end_s;
}

void NodeSRange::set_range_s(double start_s, double end_s) {
    _start_s = start_s;
    _end_s = end_s;
}

bool NodeSRange::merge_range_overlap(const NodeSRange& other) {
    if (!is_valid() || !other.is_valid()) {
        return false;
    }
    if (other.start_s() > end_s() || other.end_s() < start_s()) {
        return false;
    }
    set_end_s(std::max(end_s(), other.end_s()));
    set_start_s(std::min(start_s(), other.start_s()));
    return true;
}

NodeWithRange::NodeWithRange(const NodeSRange& range, const TopoNode* node) :
                            NodeSRange(range), _topo_node(node) { }

NodeWithRange::~NodeWithRange() { }

bool NodeWithRange::operator < (const NodeWithRange& other) const {
    return start_s() > other.start_s();
}

const TopoNode* NodeWithRange::topo_node() const {
    return _topo_node;
}

} // namespace routing
} // namespace adu

