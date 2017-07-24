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

#ifndef MODULES_PREDICTION_COMMON_ROAD_GRAPH_H_
#define MODULES_PREDICTION_COMMON_ROAD_GRAPH_H_

#include <vector>

#include "modules/map/hdmap/hdmap_common.h"
#include "modules/prediction/proto/lane_graph.pb.h"
#include "modules/common/proto/error_code.pb.h"

namespace apollo {
namespace prediction {

class RoadGraph {
 public:
  RoadGraph();

  RoadGraph(double start_s, double length,
            const apollo::hdmap::LaneInfo* lane_info_ptr);

  virtual ~RoadGraph();

  void Set(double start_s, double length,
           apollo::hdmap::LaneInfo* lane_info_ptr);

  apollo::common::ErrorCode BuildLaneGraph(LaneGraph* lane_graph);

 private:
  void ComputeLaneSequence(double accumulated_s,
                           double start_s,
                           const apollo::hdmap::LaneInfo* lane_info_ptr,
                           std::vector<LaneSegment>* lane_segments,
                           LaneGraph* lane_graph_ptr) const;

 private:
  double start_s_;
  double length_;
  const apollo::hdmap::LaneInfo* lane_info_ptr_;
};

}  // namespace prediction
}  // namespace apollo

#endif  // MODULES_PREDICTION_COMMON_ROAD_GRAPH_H_
