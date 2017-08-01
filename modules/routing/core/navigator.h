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

#ifndef BAIDU_ADU_ROUTING_CORE_NAVIGATOR_H
#define BAIDU_ADU_ROUTING_CORE_NAVIGATOR_H

#include <memory>

#include "router.pb.h"
#include "routing.pb.h"

#include "core/node_range_manager.h"

#include <unordered_set>

namespace adu {
namespace routing {

class TopoGraph;
class TopoNode;

class Navigator {
 public:
  explicit Navigator(const std::string& topo_file_path);
  ~Navigator();
  bool is_ready() const;

  // search old request to old response
  bool search_route(const ::adu::common::routing::RoutingRequest& request,
                    ::adu::common::routing::RoutingResult* response) const;

  // search new request to new response
  bool search_route(const ::adu::common::router::RoutingRequest& request,
                    ::adu::common::router::RoutingResult* response) const;

 private:
  // new request to new response
  bool generate_passage_region(
      const ::adu::common::router::RoutingRequest& request,
      const std::vector<const TopoNode*>& nodes,
      const std::unordered_set<const TopoNode*>& black_list,
      const NodeRangeManager& range_manager,
      ::adu::common::router::RoutingResult* result) const;

  // use internal generate result
  void generate_passage_region(
      const std::vector<const TopoNode*>& nodes,
      const std::unordered_set<const TopoNode*>& black_list,
      const NodeRangeManager& range_manager,
      ::adu::common::router::RoutingResult* result) const;

  // old request to old response
  bool generate_passage_region(
      const ::adu::common::routing::RoutingRequest& request,
      const std::vector<const TopoNode*>& nodes,
      const std::unordered_set<const TopoNode*>& black_list,
      const NodeRangeManager& range_manager,
      ::adu::common::routing::RoutingResult* result) const;

  void dump_debug_data(
      const std::vector<const TopoNode*>& nodes,
      const NodeRangeManager& range_manager,
      const ::adu::common::router::RoutingResult& response) const;

 private:
  bool _is_ready;
  std::unique_ptr<TopoGraph> _graph;
};

}  // namespace routing
}  // namespace adu

#endif  // BAIDU_ADU_ROUTING_CORE_NAVIGATOR_H
