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

#ifndef MODULES_ROUTING_TOPO_CREATOR_EDGE_CREATOR_H
#define MODULES_ROUTING_TOPO_CREATOR_EDGE_CREATOR_H

#include "modules/routing/proto/routing_config.pb.h"
#include "modules/routing/proto/topo_graph.pb.h"

namespace apollo {
namespace routing {

class EdgeCreator {
 public:
  static void GetPbEdge(const Node& node_from, const Node& node_to,
                        const Edge::DirectionType& type, Edge* pb_edge,
                        const RoutingConfig* routingconfig);

 private:
  static void InitEdgeInfo(const Node& node_from, const Node& node_to,
                           const Edge::DirectionType& type, Edge* pb_edge,
                           const RoutingConfig* routingconfig);
  static void InitEdgeCost(const Node& node_from, const Node& node_to,
                           const Edge::DirectionType& type, Edge* pb_edge,
                           const RoutingConfig* routingconfig);
};

}  // namespace routing
}  // namespace apollo

#endif  // MODULES_ROUTING_TOPO_CREATOR_EDGE_CREATOR_H
