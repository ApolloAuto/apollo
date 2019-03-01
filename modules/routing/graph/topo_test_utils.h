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

#include "modules/routing/graph/topo_node.h"

namespace apollo {
namespace routing {

const char TEST_MAP_VERSION[] = "1.0.1";
const char TEST_MAP_DISTRICT[] = "yongfeng";

const char TEST_L1[] = "L1";
const char TEST_L2[] = "L2";
const char TEST_L3[] = "L3";
const char TEST_L4[] = "L4";
const char TEST_L5[] = "L5";
const char TEST_L6[] = "L6";

const char TEST_R1[] = "R1";
const char TEST_R2[] = "R2";
const char TEST_R3[] = "R3";

const double TEST_LANE_LENGTH = 100.0;
const double TEST_LANE_COST = 1.1;
const double TEST_EDGE_COST = 2.2;

const double TEST_START_S = 0.0;
const double TEST_MIDDLE_S = 0.0;
const double TEST_END_S = TEST_LANE_LENGTH;

void GetNodeDetailForTest(Node* const node, const std::string& lane_id,
                          const std::string& road_id);

void GetNodeForTest(Node* const node, const std::string& lane_id,
                    const std::string& road_id);

void GetEdgeForTest(Edge* const edge, const std::string& lane_id_1,
                    const std::string& lane_id_2,
                    const Edge::DirectionType& type);

void GetGraphForTest(Graph* graph);

void GetGraph2ForTest(Graph* graph);

void GetGraph3ForTest(Graph* graph);

}  // namespace routing
}  // namespace apollo
