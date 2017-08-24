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

  const Node& PbNode() const;
  double Length() const;
  double Cost() const;
  bool IsVirtual() const;

  const std::string& LaneId() const;
  const std::string& RoadId() const;
  const ::apollo::hdmap::Curve& CentralCurve() const;
  const ::apollo::common::PointENU& AnchorPoint() const;

  const std::unordered_set<const TopoEdge*>& InFromAllEdge() const;
  const std::unordered_set<const TopoEdge*>& InFromLeftEdge() const;
  const std::unordered_set<const TopoEdge*>& InFromRightEdge() const;
  const std::unordered_set<const TopoEdge*>& InFromLeftOrRightEdge() const;
  const std::unordered_set<const TopoEdge*>& InFromPreEdge() const;
  const std::unordered_set<const TopoEdge*>& OutToAllEdge() const;
  const std::unordered_set<const TopoEdge*>& OutToLeftEdge() const;
  const std::unordered_set<const TopoEdge*>& OutToRightEdge() const;
  const std::unordered_set<const TopoEdge*>& OutToLeftOrRightEdge() const;
  const std::unordered_set<const TopoEdge*>& OutToSucEdge() const;

  const TopoEdge* GetInEdgeFrom(const TopoNode* from_node) const;
  const TopoEdge* GetOutEdgeTo(const TopoNode* to_node) const;

  const TopoNode* OriginNode() const;
  double StartS() const;
  double EndS() const;
  bool IsSubNode() const;

  void AddInEdge(const TopoEdge* edge);
  void AddOutEdge(const TopoEdge* edge);
  void SetOriginNode(const TopoNode* origin_node);
  void SetStartS(double start_s);
  void SetEndS(double end_s);

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

  const Edge& PbEdge() const;
  double Cost() const;
  const std::string& FromLaneId() const;
  const std::string& ToLaneId() const;
  TopoEdgeType Type() const;

  const TopoNode* FromNode() const;
  const TopoNode* ToNode() const;

 private:
  Edge pb_edge_;
  const TopoNode* from_node_;
  const TopoNode* to_node_;
};

}  // namespace routing
}  // namespace apollo

#endif  // MODULES_ROUTING_GRAPH_TOPO_NODE_H
