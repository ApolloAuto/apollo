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

#include "modules/common/log.h"
#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/common/prediction_map.h"

namespace apollo {
namespace prediction {

using ::apollo::common::PathPoint;
using ::apollo::common::TrajectoryPoint;
using ::apollo::common::math::LineSegment2d;
using ::apollo::common::math::Polygon2d;
using ::apollo::common::math::Vec2d;
using ::apollo::hdmap::JunctionInfo;
using ::apollo::planning::ADCTrajectory;

void ADCTrajectoryContainer::Insert(
    const ::google::protobuf::Message& message) {
  adc_lane_ids_.clear();
  adc_lane_seq_.clear();
  adc_junction_polygon_ = Polygon2d{};

  adc_trajectory_.CopyFrom(dynamic_cast<const ADCTrajectory&>(message));
  ADEBUG << "Received a planning message ["
         << adc_trajectory_.ShortDebugString() << "].";

  // Find junction
  if (IsProtected()) {
    SetJunctionPolygon();
  }
  ADEBUG << "Generate a polygon [" << adc_junction_polygon_.DebugString()
         << "].";

  // Find ADC lane sequence
  SetLaneSequence();
  ADEBUG << "Generate an ADC lane id sequence [" << ToString(adc_lane_seq_)
         << "].";
}

bool ADCTrajectoryContainer::IsPointInJunction(const PathPoint& point) const {
  if (adc_junction_polygon_.num_points() < 3) {
    return false;
  }
  bool in_polygon = adc_junction_polygon_.IsPointIn({point.x(), point.y()});

  bool on_virtual_lane = false;
  if (point.has_lane_id()) {
    on_virtual_lane = PredictionMap::IsVirtualLane(point.lane_id());
  }
  if (!on_virtual_lane) {
    on_virtual_lane = PredictionMap::OnVirtualLane({point.x(), point.y()},
                                                   FLAGS_virtual_lane_radius);
  }
  return in_polygon && on_virtual_lane;
}

bool ADCTrajectoryContainer::IsProtected() const {
  return adc_trajectory_.has_right_of_way_status() &&
         adc_trajectory_.right_of_way_status() == ADCTrajectory::PROTECTED;
}

void ADCTrajectoryContainer::SetJunctionPolygon() {
  std::shared_ptr<const JunctionInfo> junction_info(nullptr);

  for (int i = 0; i < adc_trajectory_.trajectory_point_size(); ++i) {
    double s = adc_trajectory_.trajectory_point(i).path_point().s();

    if (s > FLAGS_adc_trajectory_search_length) {
      break;
    }

    if (junction_info != nullptr) {
      break;
    }

    double x = adc_trajectory_.trajectory_point(i).path_point().x();
    double y = adc_trajectory_.trajectory_point(i).path_point().y();
    std::vector<std::shared_ptr<const JunctionInfo>> junctions =
        PredictionMap::GetJunctions({x, y}, FLAGS_junction_search_radius);
    if (!junctions.empty() && junctions.front() != nullptr) {
      junction_info = junctions.front();
    }
  }

  if (junction_info != nullptr && junction_info->junction().has_polygon()) {
    std::vector<Vec2d> vertices;
    for (const auto& point : junction_info->junction().polygon().point()) {
      vertices.emplace_back(point.x(), point.y());
    }
    if (vertices.size() >= 3) {
      adc_junction_polygon_ = Polygon2d{vertices};
    }
  }
}

void ADCTrajectoryContainer::SetLaneSequence() {
  for (const auto& lane : adc_trajectory_.lane_id()) {
    if (!lane.id().empty()) {
      if (adc_lane_seq_.empty() || lane.id() != adc_lane_seq_.back()) {
        adc_lane_seq_.emplace_back(lane.id());
      }
    }
  }
  adc_lane_ids_.clear();
  adc_lane_ids_.insert(adc_lane_seq_.begin(), adc_lane_seq_.end());
}

std::string ADCTrajectoryContainer::ToString(
    const std::unordered_set<std::string>& lane_ids) {
  std::string str_lane_sequence = "";
  auto it = lane_ids.begin();
  if (it != lane_ids.end()) {
    str_lane_sequence += (*it);
    ++it;
  }
  for (; it != lane_ids.end(); ++it) {
    str_lane_sequence += ("->" + *it);
  }
  return str_lane_sequence;
}

std::string ADCTrajectoryContainer::ToString(
    const std::vector<std::string>& lane_ids) {
  std::string str_lane_sequence = "";
  auto it = lane_ids.begin();
  if (it != lane_ids.end()) {
    str_lane_sequence += (*it);
    ++it;
  }
  for (; it != lane_ids.end(); ++it) {
    str_lane_sequence += ("->" + *it);
  }
  return str_lane_sequence;
}

bool ADCTrajectoryContainer::HasOverlap(const LaneSequence& lane_sequence) {
  for (const auto& lane_segment : lane_sequence.lane_segment()) {
    std::string lane_id = lane_segment.lane_id();
    if (adc_lane_ids_.find(lane_id) != adc_lane_ids_.end()) {
      return true;
    }
  }
  return false;
}

void ADCTrajectoryContainer::SetPosition(const Vec2d& position) {
  for (auto it = adc_lane_seq_.begin(); it != adc_lane_seq_.end(); ++it) {
    auto lane_info = PredictionMap::LaneById(*it);
    if (lane_info != nullptr && lane_info->IsOnLane(position)) {
      adc_lane_ids_.clear();
      adc_lane_ids_.insert(it, adc_lane_seq_.end());
      break;
    }
  }
  ADEBUG << "Generate an ADC lane ids [" << ToString(adc_lane_ids_) << "].";
}

}  // namespace prediction
}  // namespace apollo
