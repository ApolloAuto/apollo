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

#ifndef BAIDU_ADU_ROUTING_TOPO_CREATOR_GRAPH_CREATOR_H
#define BAIDU_ADU_ROUTING_TOPO_CREATOR_GRAPH_CREATOR_H

#include <unordered_set>
#include <unordered_map>

#include "map.pb.h"
#include "topo_graph.pb.h"

namespace adu {
namespace routing {

class GraphCreator {
public:
    GraphCreator(const std::string& base_map_file_path, const std::string& dump_topo_file_path);

    ~GraphCreator() = default;

    bool create();

private:
    void add_edge(const ::adu::routing::common::Node& from_node,
                  const ::google::protobuf::RepeatedPtrField<::adu::common::hdmap::Id>& to_node_vec,
                  const ::adu::routing::common::Edge::DirectionType& type);

private:
    std::string _base_map_file_path;
    std::string _dump_topo_file_path;
    ::adu::common::hdmap::Map _pbmap;
    ::adu::routing::common::Graph _graph;
    std::unordered_map<std::string, int> _node_index_map;
    std::unordered_map<std::string, std::string> _road_id_map;
    std::unordered_set<std::string> _showed_edge_id_set;
};

} // namespace routing
} // namespace adu

#endif // BAIDU_ADU_ROUTING_TOPO_CREATOR_GRAPH_CREATOR_H
