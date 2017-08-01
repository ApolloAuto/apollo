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

#ifndef BAIDU_ADU_ROUTING_GRAPH_TOPO_NODE_H
#define BAIDU_ADU_ROUTING_GRAPH_TOPO_NODE_H

#include <unordered_map>
#include <unordered_set>

#include "map_lane.pb.h"
#include "topo_graph.pb.h"

namespace adu {
namespace routing {

class TopoEdge;

class TopoNode {
 public:
  explicit TopoNode(const ::adu::routing::common::Node& node);
  explicit TopoNode(const TopoNode* topo_node);

  ~TopoNode();

  const ::adu::routing::common::Node& node() const;
  double length() const;
  double cost() const;
  bool is_virtual() const;

  const std::string& lane_id() const;
  const std::string& road_id() const;
  const ::adu::common::hdmap::Curve& central_curve() const;
  const ::adu::common::hdmap::Point& anchor_point() const;

  const std::unordered_set<const TopoEdge*>& in_from_all_edge() const;
  const std::unordered_set<const TopoEdge*>& in_from_left_edge() const;
  const std::unordered_set<const TopoEdge*>& in_from_right_edge() const;
  const std::unordered_set<const TopoEdge*>& in_from_left_or_right_edge() const;
  const std::unordered_set<const TopoEdge*>& in_from_pre_edge() const;
  const std::unordered_set<const TopoEdge*>& out_to_all_edge() const;
  const std::unordered_set<const TopoEdge*>& out_to_left_edge() const;
  const std::unordered_set<const TopoEdge*>& out_to_right_edge() const;
  const std::unordered_set<const TopoEdge*>& out_to_left_or_right_edge() const;
  const std::unordered_set<const TopoEdge*>& out_to_suc_edge() const;

  const TopoEdge* get_in_edge_from(const TopoNode* from_node) const;
  const TopoEdge* get_out_edge_to(const TopoNode* to_node) const;

  const TopoNode* origin_node() const;
  double start_s() const;
  double end_s() const;
  bool is_sub_node() const;

  void add_in_edge(const TopoEdge* edge);
  void add_out_edge(const TopoEdge* edge);
  void set_origin_node(const TopoNode* origin_node);
  void set_start_s(double start_s);
  void set_end_s(double end_s);

 private:
  ::adu::routing::common::Node _pb_node;
  ::adu::common::hdmap::Point _anchor_point;

  double _start_s;
  double _end_s;

  std::unordered_set<const TopoEdge*> _in_from_all_edge_set;
  std::unordered_set<const TopoEdge*> _in_from_left_edge_set;
  std::unordered_set<const TopoEdge*> _in_from_right_edge_set;
  std::unordered_set<const TopoEdge*> _in_from_left_or_right_edge_set;
  std::unordered_set<const TopoEdge*> _in_from_pre_edge_set;
  std::unordered_set<const TopoEdge*> _out_to_all_edge_set;
  std::unordered_set<const TopoEdge*> _out_to_left_edge_set;
  std::unordered_set<const TopoEdge*> _out_to_right_edge_set;
  std::unordered_set<const TopoEdge*> _out_to_left_or_right_edge_set;
  std::unordered_set<const TopoEdge*> _out_to_suc_edge_set;

  std::unordered_map<const TopoNode*, const TopoEdge*> _out_edge_map;
  std::unordered_map<const TopoNode*, const TopoEdge*> _in_edge_map;

  const TopoNode* _origin_node;
};

}  // namespace routing
}  // namespace adu

#endif  // BAIDU_ADU_ROUTING_GRAPH_TOPO_NODE_H
