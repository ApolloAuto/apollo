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

#include "core/routing_result_generator.h"

#include "ros/time.h"

#include "common/routing_gflags.h"
#include "graph/topo_node.h"

namespace adu {
namespace routing {

using ::adu::common::routing::RoutingRequest;
using ::adu::common::routing::RoutingResult;

// old request to old response
bool RoutingResultGenerator::generate_passage_region(
                                const RoutingRequest& request,
                                const std::vector<const TopoNode*>& nodes,
                                const std::unordered_set<const TopoNode*>& black_list,
                                const NodeSRangeManager& range_manager,
                                RoutingResult* result) const {
    result->mutable_header()->set_timestamp_sec(ros::Time::now().toSec());
    result->mutable_header()->set_module_name(FLAGS_node_name);
    result->mutable_header()->set_sequence_num(1);
    for (const auto& node : nodes) {
        auto* seg = result->add_route();;
        seg->set_id(node->lane_id());
        NodeSRange range = range_manager.get_node_range(node);
        seg->set_start_s(range.start_s());
        seg->set_end_s(range.end_s());
    }
    result->mutable_routing_request()->CopyFrom(request);
    return true;

}

} // namespace routing
} // namespace adu

