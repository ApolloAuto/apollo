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

#ifndef MODULES_ROUTING_GRAPH_TOPO_EDGE_H
#define MODULES_ROUTING_GRAPH_TOPO_EDGE_H

#include "map_lane.pb.h"
#include "topo_graph.pb.h"

namespace apollo {
namespace routing {

enum TopoEdgeType {
  TET_FORWARD,
  TET_LEFT,
  TET_RIGHT,
};

class TopoNode;

class TopoEdge {
 public:
  TopoEdge(const ::apollo::routing::common::Edge& edge, const TopoNode* from_node,
           const TopoNode* to_node);

  ~TopoEdge();

  const ::apollo::routing::common::Edge& edge() const;
  double cost() const;
  const std::string& from_lane_id() const;
  const std::string& to_lane_id() const;
  TopoEdgeType type() const;

  const TopoNode* from_node() const;
  const TopoNode* to_node() const;

 private:
  ::apollo::routing::common::Edge _pb_edge;
  const TopoNode* _from_node;
  const TopoNode* _to_node;
};

}  // namespace routing
}  // namespace apollo

#endif  // MODULES_ROUTING_GRAPH_TOPO_EDGE_H
