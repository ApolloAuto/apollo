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

#ifndef BAIDU_ADU_ROUTER_CORE_ROUTER_RESULT_GENERATOR_H
#define BAIDU_ADU_ROUTER_CORE_ROUTER_RESULT_GENERATOR_H

#include <memory>
#include <unordered_set>

#include "router.pb.h"

#include "graph/topo_range_manager.h"

namespace adu {
namespace routing {

class TopoGraph;
class TopoNode;

class RouterResultGenerator {
public:
    RouterResultGenerator() = default;
    ~RouterResultGenerator() = default;

    // new request to new response
    bool generate_passage_region(const std::string& map_version,
                                 const ::adu::common::router::RoutingRequest& request,
                                 const std::vector<const TopoNode*>& nodes,
                                 const std::unordered_set<const TopoNode*>& black_list,
                                 const NodeSRangeManager& range_manager,
                                 ::adu::common::router::RoutingResult* result) const;

    // use internal generate result
    void generate_passage_region(const std::vector<const TopoNode*>& nodes,
                                 const std::unordered_set<const TopoNode*>& black_list,
                                 const NodeSRangeManager& range_manager,
                                 ::adu::common::router::RoutingResult* result) const;
};

} // namespace routing
} // namespace adu

#endif // BAIDU_ADU_ROUTER_CORE_ROUTER_RESULT_GENERATOR_H

