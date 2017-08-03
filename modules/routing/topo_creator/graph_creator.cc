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

#include "modules/routing/topo_creator/graph_creator.h"

#include "glog/logging.h"

#include "modules/routing/common/utils.h"
#include "modules/routing/topo_creator/edge_creator.h"
#include "modules/routing/topo_creator/node_creator.h"
#include "modules/common/util/file.h"

namespace apollo {
namespace routing {

namespace {

using ::google::protobuf::RepeatedPtrField;

using ::apollo::hdmap::Id;

using ::apollo::routing::Node;
using ::apollo::routing::Edge;

std::string get_edge_id(const std::string& from_id, const std::string& to_id) {
  return from_id + "->" + to_id;
}

}  // namespace

GraphCreator::GraphCreator(const std::string& base_map_file_path,
                           const std::string& dump_topo_file_path)
    : _base_map_file_path(base_map_file_path),
      _dump_topo_file_path(dump_topo_file_path) {}

bool GraphCreator::create() {
  if (!::apollo::common::util::GetProtoFromFile(
          _base_map_file_path, &_pbmap)) {
    AERROR << "Failed to load base map file from " << _base_map_file_path;
    return false;
  }
  AINFO << "Number of lanes: " << _pbmap.lane_size();

  _graph.set_hdmap_version(_pbmap.header().version());
  _graph.set_hdmap_district(_pbmap.header().district());

  _node_index_map.clear();
  _road_id_map.clear();
  _showed_edge_id_set.clear();

  for (const auto& road : _pbmap.road()) {
    for (const auto& section : road.section()) {
      for (const auto& lane : section.lane_id()) {
        _road_id_map[lane.id()] = road.id().id();
      }
    }
  }

  for (const auto& lane : _pbmap.lane()) {
    AINFO << "Current lane id: " << lane.id().id();
    _node_index_map[lane.id().id()] = _graph.node_size();
    const auto iter = _road_id_map.find(lane.id().id());
    if (iter != _road_id_map.end()) {
      NodeCreator::get_pb_node(lane, iter->second, _graph.add_node());
    } else {
      LOG(WARNING) << "Failed to find road id of lane " << lane.id().id();
      NodeCreator::get_pb_node(lane, "", _graph.add_node());
    }
  }

  std::string edge_id = "";
  for (const auto& lane : _pbmap.lane()) {
    const auto& from_node = _graph.node(_node_index_map[lane.id().id()]);
    add_edge(from_node, lane.left_neighbor_forward_lane_id(), Edge::LEFT);
    add_edge(from_node, lane.right_neighbor_forward_lane_id(), Edge::RIGHT);
    add_edge(from_node, lane.successor_id(), Edge::FORWARD);
  }

  if (!::apollo::common::util::SetProtoToASCIIFile(
          _graph, _dump_topo_file_path)) {
    AERROR << "Failed to dump topo data into file " << _dump_topo_file_path;
    return false;
  }
  AINFO << "File is dumped successfully. Path: " << _dump_topo_file_path;
  return true;
}

void GraphCreator::add_edge(const Node& from_node,
                            const RepeatedPtrField<Id>& to_node_vec,
                            const Edge::DirectionType& type) {
  std::string edge_id = "";
  for (const auto& to_id : to_node_vec) {
    edge_id = get_edge_id(from_node.lane_id(), to_id.id());
    if (_showed_edge_id_set.count(edge_id) != 0) {
      continue;
    }
    _showed_edge_id_set.insert(edge_id);
    const auto& iter = _node_index_map.find(to_id.id());
    if (iter == _node_index_map.end()) {
      continue;
    }
    const auto& to_node = _graph.node(iter->second);
    EdgeCreator::get_pb_edge(from_node, to_node, type, _graph.add_edge());
  }
}

}  // namespace routing
}  // namespace apollo
