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

#include "modules/routing/graph/topo_test_utils.h"

namespace apollo {
namespace routing {

namespace {

using apollo::hdmap::Curve;

void AddCurve(Curve* curve) {
  auto* curve_segment = curve->add_segment();
  curve_segment->set_s(TEST_START_S);
  curve_segment->mutable_start_position()->set_x(0.0);
  curve_segment->mutable_start_position()->set_y(0.0);
  curve_segment->set_heading(0.0);
  curve_segment->set_length(TEST_LANE_LENGTH);
  auto* lane_segment = curve_segment->mutable_line_segment();
  auto* point1 = lane_segment->add_point();
  point1->set_x(TEST_START_S);
  point1->set_y(0.0);
  auto* point21 = lane_segment->add_point();
  point21->set_x(TEST_MIDDLE_S / 2);
  point21->set_y(0.0);
  auto* point2 = lane_segment->add_point();
  point2->set_x(TEST_MIDDLE_S);
  point2->set_y(0.0);
  auto* point22 = lane_segment->add_point();
  point22->set_x(TEST_MIDDLE_S + TEST_MIDDLE_S / 2);
  point22->set_y(0.0);
  auto* point3 = lane_segment->add_point();
  point3->set_x(TEST_END_S);
  point3->set_y(0.0);
}

}  // namespace

void GetNodeDetailForTest(Node* const node, const std::string& lane_id,
                          const std::string& road_id) {
  node->set_lane_id(lane_id);
  node->set_length(TEST_LANE_LENGTH);
  node->set_road_id(road_id);
  node->set_cost(TEST_LANE_COST);
  AddCurve(node->mutable_central_curve());
  auto* left_out = node->add_left_out();
  left_out->mutable_start()->set_s(TEST_START_S);
  left_out->mutable_end()->set_s(TEST_END_S);
  auto* right_out = node->add_right_out();
  right_out->mutable_start()->set_s(TEST_START_S);
  right_out->mutable_end()->set_s(TEST_END_S);
}

void GetNodeForTest(Node* const node, const std::string& lane_id,
                    const std::string& road_id) {
  node->set_lane_id(lane_id);
  node->set_length(TEST_LANE_LENGTH);
  node->set_road_id(road_id);
  node->set_cost(TEST_LANE_COST);
  auto* left_out = node->add_left_out();
  left_out->mutable_start()->set_s(TEST_START_S);
  left_out->mutable_end()->set_s(TEST_END_S);
  auto* right_out = node->add_right_out();
  right_out->mutable_start()->set_s(TEST_START_S);
  right_out->mutable_end()->set_s(TEST_END_S);
}

void GetEdgeForTest(Edge* const edge, const std::string& lane_id_1,
                    const std::string& lane_id_2,
                    const Edge::DirectionType& type) {
  edge->set_from_lane_id(lane_id_1);
  edge->set_to_lane_id(lane_id_2);
  edge->set_cost(TEST_EDGE_COST);
  edge->set_direction_type(type);
}

void GetGraphForTest(Graph* graph) {
  graph->set_hdmap_version(TEST_MAP_VERSION);
  graph->set_hdmap_district(TEST_MAP_DISTRICT);
  GetNodeForTest(graph->add_node(), TEST_L1, TEST_R1);
  GetNodeForTest(graph->add_node(), TEST_L2, TEST_R1);
  GetNodeForTest(graph->add_node(), TEST_L3, TEST_R2);
  GetNodeForTest(graph->add_node(), TEST_L4, TEST_R2);

  GetEdgeForTest(graph->add_edge(), TEST_L1, TEST_L2, Edge::RIGHT);
  GetEdgeForTest(graph->add_edge(), TEST_L2, TEST_L1, Edge::LEFT);
  GetEdgeForTest(graph->add_edge(), TEST_L3, TEST_L4, Edge::RIGHT);
  GetEdgeForTest(graph->add_edge(), TEST_L4, TEST_L3, Edge::LEFT);
  GetEdgeForTest(graph->add_edge(), TEST_L1, TEST_L3, Edge::FORWARD);
  GetEdgeForTest(graph->add_edge(), TEST_L2, TEST_L4, Edge::FORWARD);
}

void GetGraph2ForTest(Graph* graph) {
  graph->set_hdmap_version(TEST_MAP_VERSION);
  graph->set_hdmap_district(TEST_MAP_DISTRICT);
  GetNodeForTest(graph->add_node(), TEST_L1, TEST_R1);
  GetNodeForTest(graph->add_node(), TEST_L2, TEST_R1);
  GetNodeForTest(graph->add_node(), TEST_L3, TEST_R2);
  GetNodeForTest(graph->add_node(), TEST_L4, TEST_R2);
  GetNodeForTest(graph->add_node(), TEST_L5, TEST_R2);
  GetNodeForTest(graph->add_node(), TEST_L6, TEST_R3);

  GetEdgeForTest(graph->add_edge(), TEST_L1, TEST_L2, Edge::RIGHT);
  GetEdgeForTest(graph->add_edge(), TEST_L2, TEST_L1, Edge::LEFT);
  GetEdgeForTest(graph->add_edge(), TEST_L3, TEST_L4, Edge::RIGHT);
  GetEdgeForTest(graph->add_edge(), TEST_L4, TEST_L3, Edge::LEFT);
  GetEdgeForTest(graph->add_edge(), TEST_L4, TEST_L5, Edge::RIGHT);
  GetEdgeForTest(graph->add_edge(), TEST_L5, TEST_L4, Edge::LEFT);
  GetEdgeForTest(graph->add_edge(), TEST_L1, TEST_L3, Edge::FORWARD);
  GetEdgeForTest(graph->add_edge(), TEST_L2, TEST_L4, Edge::FORWARD);
  GetEdgeForTest(graph->add_edge(), TEST_L5, TEST_L6, Edge::FORWARD);
}

void GetGraph3ForTest(Graph* graph) {
  graph->set_hdmap_version(TEST_MAP_VERSION);
  graph->set_hdmap_district(TEST_MAP_DISTRICT);
  GetNodeForTest(graph->add_node(), TEST_L1, TEST_R1);
  GetNodeForTest(graph->add_node(), TEST_L2, TEST_R1);
  GetNodeForTest(graph->add_node(), TEST_L3, TEST_R2);
  GetNodeForTest(graph->add_node(), TEST_L4, TEST_R2);
  GetNodeForTest(graph->add_node(), TEST_L5, TEST_R3);
  GetNodeForTest(graph->add_node(), TEST_L6, TEST_R3);

  GetEdgeForTest(graph->add_edge(), TEST_L1, TEST_L2, Edge::RIGHT);
  GetEdgeForTest(graph->add_edge(), TEST_L2, TEST_L1, Edge::LEFT);
  GetEdgeForTest(graph->add_edge(), TEST_L3, TEST_L4, Edge::RIGHT);
  GetEdgeForTest(graph->add_edge(), TEST_L4, TEST_L3, Edge::LEFT);
  GetEdgeForTest(graph->add_edge(), TEST_L5, TEST_L6, Edge::RIGHT);
  GetEdgeForTest(graph->add_edge(), TEST_L6, TEST_L5, Edge::LEFT);
  GetEdgeForTest(graph->add_edge(), TEST_L1, TEST_L3, Edge::FORWARD);
  GetEdgeForTest(graph->add_edge(), TEST_L3, TEST_L5, Edge::FORWARD);
  GetEdgeForTest(graph->add_edge(), TEST_L2, TEST_L4, Edge::FORWARD);
  GetEdgeForTest(graph->add_edge(), TEST_L4, TEST_L6, Edge::FORWARD);
}

}  // namespace routing
}  // namespace apollo
