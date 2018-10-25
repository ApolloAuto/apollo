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

#pragma once

#include <string>
#include <unordered_map>
#include <unordered_set>

#include "modules/map/proto/map.pb.h"
#include "modules/routing/proto/routing_config.pb.h"
#include "modules/routing/proto/topo_graph.pb.h"

namespace apollo {
namespace routing {

class GraphCreator {
 public:
  GraphCreator(const std::string& base_map_file_path,
               const std::string& dump_topo_file_path,
               const RoutingConfig& routing_conf);

  ~GraphCreator() = default;

  bool Create();

 private:
  void InitForbiddenLanes();
  std::string GetEdgeID(const std::string& from_id, const std::string& to_id);

  void AddEdge(
      const Node& from_node,
      const ::google::protobuf::RepeatedPtrField<hdmap::Id>& to_node_vec,
      const Edge::DirectionType& type);

  static bool IsValidUTurn(const hdmap::Lane& lane, const double radius);

 private:
  std::string base_map_file_path_;
  std::string dump_topo_file_path_;
  hdmap::Map pbmap_;
  Graph graph_;
  std::unordered_map<std::string, int> node_index_map_;
  std::unordered_map<std::string, std::string> road_id_map_;
  std::unordered_set<std::string> showed_edge_id_set_;
  std::unordered_set<std::string> forbidden_lane_id_set_;

  const RoutingConfig& routing_conf_;
};

}  // namespace routing
}  // namespace apollo
