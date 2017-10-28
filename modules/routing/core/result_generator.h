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
  bool GeneratePassageRegion(const std::vector<NodeWithRange>& nodes,
                             const TopoRangeManager& range_manager,
                             RoutingResponse* const result);
};

}  // namespace routing
}  // namespace apollo

#endif  // MODULES_ROUTING_CORE_RESULT_GENERATOR_H_
