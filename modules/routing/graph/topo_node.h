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

#ifndef MODULES_ROUTING_GRAPH_TOPO_NODE_H
#define MODULES_ROUTING_GRAPH_TOPO_NODE_H

#include <unordered_map>
#include <unordered_set>

#include "modules/map/proto/map_lane.pb.h"
#include "modules/routing/proto/topo_graph.pb.h"

namespace apollo {
namespace routing {

class TopoEdge;

class TopoNode {
 public:
  explicit TopoNode(const Node& node);
  explicit TopoNode(const TopoNode* topo_node);

  ~TopoNode();

  const Node& node() const;
  double length() const;
  double cost() const;
  bool is_virtual() const;

  const std::string& lane_id() const;
  const std::string& road_id() const;
  const ::apollo::hdmap::Curve& central_curve() const;
  const ::apollo::common::PointENU& anchor_point() const;

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
  void setorigin_node_(const TopoNode* origin_node);
  void set_start_s(double start_s);
  void set_end_s(double end_s);

 private:
  Node pb_node_;
  ::apollo::common::PointENU anchor_point_;

  double start_s_;
  double end_s_;

  std::unordered_set<const TopoEdge*> in_from_all_edge_set_;
  std::unordered_set<const TopoEdge*> in_from_left_edge_set_;
  std::unordered_set<const TopoEdge*> in_from_right_edge_set_;
  std::unordered_set<const TopoEdge*> in_from_left_or_right_edge_set_;
  std::unordered_set<const TopoEdge*> in_from_pre_edge_set_;
  std::unordered_set<const TopoEdge*> out_to_all_edge_set_;
  std::unordered_set<const TopoEdge*> out_to_left_edge_set_;
  std::unordered_set<const TopoEdge*> out_to_right_edge_set_;
  std::unordered_set<const TopoEdge*> out_to_left_or_right_edge_set_;
  std::unordered_set<const TopoEdge*> out_to_suc_edge_set_;

  std::unordered_map<const TopoNode*, const TopoEdge*> out_edge_map_;
  std::unordered_map<const TopoNode*, const TopoEdge*> in_edge_map_;

  const TopoNode* origin_node_;
};

enum TopoEdgeType {
  TET_FORWARD,
  TET_LEFT,
  TET_RIGHT,
};

class TopoEdge {
 public:
  TopoEdge(const Edge& edge, const TopoNode* from_node,
           const TopoNode* to_node);

  ~TopoEdge();

  const Edge& edge() const;
  double cost() const;
  const std::string& from_lane_id() const;
  const std::string& to_lane_id() const;
  TopoEdgeType type() const;

  const TopoNode* from_node() const;
  const TopoNode* to_node() const;

 private:
  Edge pb_edge_;
  const TopoNode* from_node_;
  const TopoNode* to_node_;
};

}  // namespace routing
}  // namespace apollo

#endif  // MODULES_ROUTING_GRAPH_TOPO_NODE_H
