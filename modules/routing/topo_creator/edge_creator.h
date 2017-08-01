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

#ifndef BAIDU_ADU_ROUTING_TOPO_CREATOR_EDGE_CREATOR_H
#define BAIDU_ADU_ROUTING_TOPO_CREATOR_EDGE_CREATOR_H

#include "topo_graph.pb.h"

namespace adu {
namespace routing {

class EdgeCreator {
 public:
  static void get_pb_edge(
      const ::adu::routing::common::Node& node_from,
      const ::adu::routing::common::Node& node_to,
      const ::adu::routing::common::Edge::DirectionType& type,
      ::adu::routing::common::Edge* pb_edge);

 private:
  static void init_edge_info(
      const ::adu::routing::common::Node& node_from,
      const ::adu::routing::common::Node& node_to,
      const ::adu::routing::common::Edge::DirectionType& type,
      ::adu::routing::common::Edge* pb_edge);
  static void init_edge_cost(
      const ::adu::routing::common::Node& node_from,
      const ::adu::routing::common::Node& node_to,
      const ::adu::routing::common::Edge::DirectionType& type,
      ::adu::routing::common::Edge* pb_edge);
};

}  // namespace routing
}  // namespace adu

#endif  // BAIDU_ADU_ROUTING_TOPO_CREATOR_EDGE_CREATOR_H
