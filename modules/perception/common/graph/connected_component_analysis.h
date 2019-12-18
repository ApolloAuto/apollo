/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include <queue>
#include <vector>

#include "cyber/common/log.h"

namespace apollo {
namespace perception {
namespace common {

/*
 * @brief: bfs based connected component analysis
 * @params[IN] graph: input graph for connected component analysis
 * @params[OUT] components: connected components of input graph
 * @return nothing
 * */
void ConnectedComponentAnalysis(const std::vector<std::vector<int>>& graph,
                                std::vector<std::vector<int>>* components);

}  // namespace common
}  // namespace perception
}  // namespace apollo
