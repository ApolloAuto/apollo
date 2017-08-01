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

#ifndef BAIDU_ADU_ROUTING_STRATEGY_A_STAR_STRATEGY_H
#define BAIDU_ADU_ROUTING_STRATEGY_A_STAR_STRATEGY_H

#include <unordered_map>
#include <unordered_set>

#include "strategy/strategy.h"

namespace adu {
namespace routing {

class AStarStrategy : public Strategy {
 public:
  AStarStrategy() = default;
  ~AStarStrategy() = default;

  virtual bool search(const TopoGraph* graph, const TopoNode* src_node,
                      const TopoNode* dest_node,
                      const std::unordered_set<const TopoNode*>& black_list,
                      std::vector<const TopoNode*>* const result_nodes);

 private:
  void clear();
  double heuristic_cost(const TopoNode* src_node, const TopoNode* dest_node);

 private:
  std::unordered_set<const TopoNode*> _open_set;
  std::unordered_set<const TopoNode*> _closed_set;
  std::unordered_map<const TopoNode*, const TopoNode*> _came_from;
  std::unordered_map<const TopoNode*, double> _g_score;
};

}  // namespace routing
}  // namespace adu

#endif  // BAIDU_ADU_ROUTING_STRATEGY_A_STAR_STRATEGY_H
