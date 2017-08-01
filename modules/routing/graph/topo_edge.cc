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

#include "graph/topo_edge.h"

#include <math.h>

#include "graph/topo_node.h"

namespace adu {
namespace routing {

using ::adu::routing::common::Edge;

TopoEdge::TopoEdge(const ::adu::routing::common::Edge& edge,
                   const TopoNode* from_node, const TopoNode* to_node)
    : _pb_edge(edge), _from_node(from_node), _to_node(to_node) {}

TopoEdge::~TopoEdge() {}

const Edge& TopoEdge::edge() const { return _pb_edge; }

double TopoEdge::cost() const { return _pb_edge.cost(); }

const TopoNode* TopoEdge::from_node() const { return _from_node; }

const TopoNode* TopoEdge::to_node() const { return _to_node; }

const std::string& TopoEdge::from_lane_id() const {
  return _pb_edge.from_lane_id();
}

const std::string& TopoEdge::to_lane_id() const {
  return _pb_edge.to_lane_id();
}

TopoEdgeType TopoEdge::type() const {
  if (_pb_edge.direction_type() == ::adu::routing::common::Edge::LEFT) {
    return TET_LEFT;
  }
  if (_pb_edge.direction_type() == ::adu::routing::common::Edge::RIGHT) {
    return TET_RIGHT;
  }
  return TET_FORWARD;
}

}  // namespace routing
}  // namespace adu
