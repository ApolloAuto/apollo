/******************************************************************************
 * Copyright 2021 The Apollo Authors. All Rights Reserved.
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
#include "modules/prediction/pipeline/vector_net.h"

#include <vector>

#include "cyber/common/file.h"
#include "modules/map/hdmap/hdmap_util.h"

namespace apollo {
namespace prediction {
bool VectorNet::query_nearby_map(const double obstacle_x,
                                 const double obstacle_y,
                                 const double obstacle_phi) {
  apollo::hdmap::HDMapUtil::ReloadMaps();
  vector_net_pb_.mutable_car_position()->set_x(obstacle_x);
  vector_net_pb_.mutable_car_position()->set_y(obstacle_y);
  GetRoads(obstacle_x, obstacle_y);
  cyber::common::SetProtoToASCIIFile(vector_net_pb_,
                                     FLAGS_prediction_target_file);
  AINFO << "Obstacle heading." << obstacle_phi;

  return true;
}

void VectorNet::GetRoads(const double base_x, const double base_y) {
  common::PointENU center_point;
  center_point.set_x(base_x);
  center_point.set_y(base_x);

  std::vector<apollo::hdmap::RoadInfoConstPtr> roads;
  apollo::hdmap::HDMapUtil::BaseMap().GetRoads(center_point, 141.4, &roads);
  AINFO << "Road Size: " << roads.size();

  for (const auto& road : roads) {
    auto road_ = vector_net_pb_.add_road();
    road_->set_id(road->road().id().id());
    for (const auto& section : road->road().section()) {
      for (const auto& edge : section.boundary().outer_polygon().edge()) {
        if (edge.type() == 2) {  // left edge
          for (const auto& segment : edge.curve().segment()) {
            auto edge_ = road_->add_edge();
            auto size = segment.line_segment().point().size();
            auto first = segment.line_segment().point(0);
            auto last = segment.line_segment().point(size - 1);
            edge_->set_type(apollo::prediction::Edge::LEFT_BOUNDARY);
            edge_->mutable_start_point()->CopyFrom(first);
            edge_->mutable_end_point()->CopyFrom(last);
          }
        } else if (edge.type() == 3) {  // right edge
          for (const auto& segment : edge.curve().segment()) {
            auto edge_ = road_->add_edge();
            auto size = segment.line_segment().point().size();
            auto first = segment.line_segment().point(0);
            auto last = segment.line_segment().point(size - 1);
            edge_->set_type(apollo::prediction::Edge::RIGHT_BOUNDARY);
            edge_->mutable_start_point()->CopyFrom(last);
            edge_->mutable_end_point()->CopyFrom(first);
          }
        }
      }
    }
  }
}
}  // namespace prediction
}  // namespace apollo
