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

#ifndef MODULES_PREDICTION_CONTAINER_OBSTACLES_OBSTACLE_CLUSTERS_H_
#define MODULES_PREDICTION_CONTAINER_OBSTACLES_OBSTACLE_CLUSTERS_H_

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>

#include "modules/common/macro.h"
#include "modules/map/hdmap/hdmap_common.h"
#include "modules/prediction/proto/lane_graph.pb.h"

namespace apollo {
namespace prediction {

class ObstacleClusters {
 public:
  /**
   * @brief Remove all lane graphs
   */
  static void Init();

  /**
   * @brief Obtain a lane graph given a lane info and s
   * @param lane start s
   * @param lane total length
   * @param lane info
   * @return a corresponding lane graph
   */
  static const LaneGraph& GetLaneGraph(
      const double start_s, const double length,
      std::shared_ptr<const apollo::hdmap::LaneInfo> lane_info_ptr);

 private:
  ObstacleClusters() = delete;

  static void Clear();

 private:
  static std::unordered_map<std::string, LaneGraph> lane_graphs_;
};

}  // namespace prediction
}  // namespace apollo

#endif  // MODULES_PREDICTION_CONTAINER_OBSTACLES_OBSTACLE_CLUSTERS_H_
