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

#ifndef MODULES_ROUTING_CORE_RESULT_GENERATOR_H_
#define MODULES_ROUTING_CORE_RESULT_GENERATOR_H_

#include <string>
#include <utility>
#include <vector>

#include "modules/routing/proto/routing.pb.h"

#include "modules/routing/graph/node_with_range.h"
#include "modules/routing/graph/topo_graph.h"
#include "modules/routing/graph/topo_range_manager.h"

namespace apollo {
namespace routing {

class ResultGenerator {
 public:
  ResultGenerator() = default;
  ~ResultGenerator() = default;

  bool GeneratePassageRegion(const std::string& map_version,
                             const RoutingRequest& request,
                             const std::vector<NodeWithRange>& nodes,
                             const TopoRangeManager& range_manager,
                             RoutingResponse* const result);

 private:
  struct PassageInfo {
    std::vector<NodeWithRange> nodes;
    ChangeLaneType change_lane_type;
    PassageInfo() = default;
    PassageInfo(const std::vector<NodeWithRange>& _nodes,
                ChangeLaneType _change_lane_type)
        : nodes(_nodes), change_lane_type(_change_lane_type) {}
  };

  bool GeneratePassageRegion(const std::vector<NodeWithRange>& nodes,
                             const TopoRangeManager& range_manager,
                             RoutingResponse* const result);

  void CreateRoadSegments(const std::vector<PassageInfo>& passages,
                          RoutingResponse* result);

  void AddRoadSegment(const std::vector<PassageInfo>& passages,
                      const std::pair<std::size_t, std::size_t>& start,
                      const std::pair<std::size_t, std::size_t>& end,
                      RoutingResponse* result);
  void ExtendPassages(const TopoRangeManager& range_manager,
                      std::vector<PassageInfo>* const passages);
  bool ExtractBasicPassages(const std::vector<NodeWithRange>& nodes,
                            std::vector<PassageInfo>* const passsages);
  void ExtendBackward(const TopoRangeManager& range_manager,
                      const PassageInfo& prev_passage,
                      PassageInfo* const curr_passage);
  void ExtendForward(const TopoRangeManager& range_manager,
                     const PassageInfo& next_passage,
                     PassageInfo* const curr_passage);
  bool IsReachableFromWithChangeLane(const TopoNode* from_node,
                                     const PassageInfo& to_nodes,
                                     NodeWithRange* reachable_node);
  bool IsReachableToWithChangeLane(const TopoNode* from_node,
                                   const PassageInfo& to_nodes,
                                   NodeWithRange* reachable_node);
};

}  // namespace routing
}  // namespace apollo

#endif  // MODULES_ROUTING_CORE_RESULT_GENERATOR_H_
