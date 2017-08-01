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

#ifndef BAIDU_ADU_ROUTING_TEST_TEST_TOOL_TEST_UTILS_H
#define BAIDU_ADU_ROUTING_TEST_TEST_TOOL_TEST_UTILS_H

#include <string>

#include "graph/topo_graph.h"
#include "graph/topo_node.h"
#include "graph/topo_edge.h"
#include "common/utils.h"

#include "topo_graph.pb.h"
#include "gtest/gtest.h"

namespace adu {
namespace routing {

const std::string TEST_MAP_VERSION = "1.0.1";
const std::string TEST_MAP_DISTRICT = "yongfeng";

const std::string TEST_L1 = "L1";
const std::string TEST_L2 = "L2";
const std::string TEST_L3 = "L3";
const std::string TEST_L4 = "L4";
const std::string TEST_L5 = "L5";
const std::string TEST_L6 = "L6";

const std::string TEST_R1 = "R1";
const std::string TEST_R2 = "R2";
const std::string TEST_R3 = "R3";

const double TEST_LANE_LENGTH = 100.0;
const double TEST_LANE_COST = 1.1;
const double TEST_EDGE_COST = 2.2;

const double TEST_START_S = 0.0;
const double TEST_MIDDLE_S = 0.0;
const double TEST_END_S = TEST_LANE_LENGTH;

void get_node_detail_for_test(::adu::routing::common::Node* const node,
                              const std::string& lane_id,
                              const std::string& road_id);

void get_node_for_test(::adu::routing::common::Node* const node,
                       const std::string& lane_id,
                       const std::string& road_id);

void get_edge_for_test(::adu::routing::common::Edge* const edge,
                       const std::string& lane_id_1,
                       const std::string& lane_id_2,
                       const ::adu::routing::common::Edge::DirectionType& type);

void get_graph_for_test(::adu::routing::common::Graph* graph);

void get_graph_2_for_test(::adu::routing::common::Graph* graph);

} // namespace routing
} // namespace adu

#endif // BAIDU_ADU_ROUTING_TEST_TEST_TOOL_TEST_UTILS_H

