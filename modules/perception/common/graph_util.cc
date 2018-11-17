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

#include "modules/perception/common/graph_util.h"

#include <cstdio>
#include <queue>

namespace apollo {
namespace perception {

void ConnectedComponentAnalysis(const std::vector<std::vector<int>>& graph,
                                std::vector<std::vector<int>>* components) {
  int no_item = graph.size();
  std::vector<int> visited;
  visited.resize(no_item, 0);
  std::queue<int> que;
  std::vector<int> component;
  components->clear();

  for (int i = 0; i < no_item; ++i) {
    if (visited[i]) {
      continue;
    }
    component.push_back(i);
    que.push(i);
    visited[i] = 1;
    while (!que.empty()) {
      int id = que.front();
      que.pop();
      for (size_t j = 0; j < graph[id].size(); ++j) {
        int nb_id = graph[id][j];
        if (visited[nb_id] == 0) {
          component.push_back(nb_id);
          que.push(nb_id);
          visited[nb_id] = 1;
        }
      }
    }
    components->push_back(component);
    component.clear();
  }
}

}  // namespace perception
}  // namespace apollo
