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

#include "modules/prediction/container/adc_trajectory/adc_trajectory_container.h"

#include <memory>
#include <vector>

#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/common/prediction_map.h"

namespace apollo {
namespace prediction {

using apollo::common::PathPoint;
using apollo::common::TrajectoryPoint;
using apollo::common::math::LineSegment2d;
using apollo::common::math::Polygon2d;
using apollo::common::math::Vec2d;
using apollo::hdmap::JunctionInfo;
using apollo::planning::ADCTrajectory;

void ADCTrajectoryContainer::Insert(
    const ::google::protobuf::Message& message) {
  reference_line_lane_ids_.clear();
  adc_trajectory_.CopyFrom(dynamic_cast<const ADCTrajectory&>(message));
  if (!IsProtected()) {
    junction_polygon_ = Polygon2d{};
    return;
  }
  junction_polygon_ = GetJunctionPolygon();
}

bool ADCTrajectoryContainer::IsPointInJunction(const PathPoint& point) const {
  if (junction_polygon_.num_points() < 3) {
    return false;
  }
  bool in_polygon = junction_polygon_.IsPointIn({point.x(), point.y()});

  PredictionMap* map = PredictionMap::instance();
  bool on_virtual_lane = false;
  if (point.has_lane_id()) {
    on_virtual_lane = map->IsVirtualLane(point.lane_id());
  }
  if (!on_virtual_lane) {
    on_virtual_lane =
        map->OnVirtualLane({point.x(), point.y()}, FLAGS_virtual_lane_radius);
  }
  return in_polygon && on_virtual_lane;
}

bool ADCTrajectoryContainer::IsProtected() const {
  return adc_trajectory_.has_right_of_way_status() &&
         adc_trajectory_.right_of_way_status() == ADCTrajectory::PROTECTED;
}

Polygon2d ADCTrajectoryContainer::GetJunctionPolygon() {
  std::shared_ptr<const JunctionInfo> junction_info(nullptr);
  for (int i = 0; i < adc_trajectory_.trajectory_point_size(); ++i) {
    double s = adc_trajectory_.trajectory_point(i).path_point().s();
    std::string lane_id =
        adc_trajectory_.trajectory_point(i).path_point().lane_id();

    // Find junctions
    if (junction_info == nullptr && s < FLAGS_adc_trajectory_search_length) {
      double x = adc_trajectory_.trajectory_point(i).path_point().x();
      double y = adc_trajectory_.trajectory_point(i).path_point().y();
      std::vector<std::shared_ptr<const JunctionInfo>> junctions =
          PredictionMap::instance()->GetJunctions({x, y},
                                                  FLAGS_junction_search_radius);
      if (!junctions.empty() && junctions.front() != nullptr) {
        junction_info = junctions.front();
      }
    }

    // Insert reference lane ids
    if (reference_line_lane_ids_.empty() ||
        lane_id != reference_line_lane_ids_.back()) {
      reference_line_lane_ids_.emplace_back(lane_id);
    }
  }

  if (junction_info != nullptr) {
    std::vector<Vec2d> vertices;
    for (const auto& point : junction_info->junction().polygon().point()) {
      vertices.emplace_back(point.x(), point.y());
    }
    if (vertices.size() >= 3) {
      return Polygon2d{vertices};
    }
  }
  return Polygon2d{};
}

const std::vector<std::string>&
ADCTrajectoryContainer::get_reference_line_lane_ids() {
  return reference_line_lane_ids_;
}

}  // namespace prediction
}  // namespace apollo
