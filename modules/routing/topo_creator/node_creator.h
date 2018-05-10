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

#ifndef MODULES_ROUTING_TOPO_CREATOR_NODE_CREATOR_H
#define MODULES_ROUTING_TOPO_CREATOR_NODE_CREATOR_H

#include <string>

#include "modules/map/proto/map_lane.pb.h"
#include "modules/routing/proto/routing_config.pb.h"
#include "modules/routing/proto/topo_graph.pb.h"

namespace apollo {
namespace routing {

class NodeCreator {
 public:
  static void GetPbNode(const hdmap::Lane& lane, const std::string& road_id,
                        Node* pb_node, const RoutingConfig* routingconfig);

 private:
  static void AddOutBoundary(
      const hdmap::LaneBoundary& bound, double lane_length,
      ::google::protobuf::RepeatedPtrField<CurveRange>* const out_range);

  static void InitNodeInfo(const hdmap::Lane& lane, const std::string& road_id,
                           Node* const node,
                           const RoutingConfig* routingconfig);
  static void InitNodeCost(const hdmap::Lane& lane, Node* const node,
                           const RoutingConfig* routingconfig);
};

}  // namespace routing
}  // namespace apollo

#endif  // MODULES_ROUTING_TOPO_CREATOR_NODE_CREATOR_H
