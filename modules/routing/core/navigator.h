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

#include <memory>
#include <string>
#include <vector>

#include "modules/routing/core/black_list_range_generator.h"
#include "modules/routing/core/result_generator.h"

namespace apollo {
namespace routing {

class Navigator {
 public:
  explicit Navigator(const std::string& topo_file_path);
  ~Navigator();

  bool IsReady() const;

  bool SearchRoute(const routing::RoutingRequest& request,
                   routing::RoutingResponse* const response);

 private:
  bool Init(const routing::RoutingRequest& request, const TopoGraph* graph,
            std::vector<const TopoNode*>* const way_nodes,
            std::vector<double>* const way_s);

  void Clear();

  bool SearchRouteByStrategy(
      const TopoGraph* graph, const std::vector<const TopoNode*>& way_nodes,
      const std::vector<double>& way_s,
      std::vector<NodeWithRange>* const result_nodes) const;

  bool MergeRoute(const std::vector<NodeWithRange>& node_vec,
                  std::vector<NodeWithRange>* const result_node_vec) const;

 private:
  bool is_ready_ = false;
  std::unique_ptr<TopoGraph> graph_;

  TopoRangeManager topo_range_manager_;

  std::unique_ptr<BlackListRangeGenerator> black_list_generator_;
  std::unique_ptr<ResultGenerator> result_generator_;
};

}  // namespace routing
}  // namespace apollo
