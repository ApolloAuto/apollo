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

#include "modules/routing/graph/node_with_range.h"

namespace apollo {
namespace routing {

NodeWithRange::NodeWithRange(const TopoNode* node, const NodeSRange& range)
    : NodeSRange(range), topo_node_(node) {}

NodeWithRange::NodeWithRange(const TopoNode* node, double start_s, double end_s)
    : NodeSRange(start_s, end_s), topo_node_(node) {}

NodeWithRange::~NodeWithRange() {}

bool NodeWithRange::operator<(const NodeWithRange& other) const {
  return StartS() > other.StartS();
}

const TopoNode* NodeWithRange::GetTopoNode() const { return topo_node_; }

bool NodeWithRange::IsVirtual() const { return topo_node_->IsVirtual(); }

const std::string& NodeWithRange::RoadId() const {
  return topo_node_->RoadId();
}

const std::string& NodeWithRange::LaneId() const {
  return topo_node_->LaneId();
}

double NodeWithRange::FullLength() const { return topo_node_->Length(); }

}  // namespace routing
}  // namespace apollo
