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

#include <unordered_map>
#include <vector>

#include "modules/routing/graph/topo_node.h"

namespace apollo {
namespace routing {

class TopoRangeManager {
 public:
  TopoRangeManager() = default;
  virtual ~TopoRangeManager() = default;

  const std::unordered_map<const TopoNode*, std::vector<NodeSRange>>& RangeMap()
      const;
  const std::vector<NodeSRange>* Find(const TopoNode* node) const;
  void PrintDebugInfo() const;

  void Clear();
  void Add(const TopoNode* node, double start_s, double end_s);
  void SortAndMerge();

 private:
  std::unordered_map<const TopoNode*, std::vector<NodeSRange>> range_map_;
};

}  // namespace routing
}  // namespace apollo
