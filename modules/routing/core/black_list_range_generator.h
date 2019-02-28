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

#include <unordered_set>

#include "modules/routing/graph/topo_graph.h"
#include "modules/routing/graph/topo_range_manager.h"
#include "modules/routing/proto/routing.pb.h"

namespace apollo {
namespace routing {

class BlackListRangeGenerator {
 public:
  BlackListRangeGenerator() = default;
  ~BlackListRangeGenerator() = default;

  void GenerateBlackMapFromRequest(const RoutingRequest& request,
                                   const TopoGraph* graph,
                                   TopoRangeManager* const range_manager) const;

  void AddBlackMapFromTerminal(const TopoNode* src_node,
                               const TopoNode* dest_node, double start_s,
                               double end_s,
                               TopoRangeManager* const range_manager) const;
};

}  // namespace routing
}  // namespace apollo
