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

namespace apollo {
namespace prediction {

using apollo::planning::ADCTrajectory;
using apollo::common::math::LineSegment2d;
using apollo::common::TrajectoryPoint;
using apollo::common::PathPoint;
using apollo::common::math::Vec2d;

std::mutex ADCTrajectoryContainer::g_mutex_;

void ADCTrajectoryContainer::Insert(
    const ::google::protobuf::Message& message) {
  std::lock_guard<std::mutex> lock(g_mutex_);
  adc_trajectory_ = dynamic_cast<const ADCTrajectory&>(message);
}

const ADCTrajectory* ADCTrajectoryContainer::GetADCTrajectory() {
  std::lock_guard<std::mutex> lock(g_mutex_);
  return &adc_trajectory_;
}

std::vector<LineSegment2d>
ADCTrajectoryContainer::ADCTrajectorySegments(const double time_step) const {
  std::vector<LineSegment2d> segments;
  size_t num_point = adc_trajectory_.trajectory_point_size();
  if (num_point == 0) {
    return segments;
  }
  TrajectoryPoint prev_point = adc_trajectory_.trajectory_point(0);
  double prev_time = prev_point.relative_time();
  for (size_t i = 1; i < num_point; ++i) {
    TrajectoryPoint curr_point = adc_trajectory_.trajectory_point(i);
    double curr_time = curr_point.relative_time();
    if (i != num_point - 1 && curr_time - prev_time < time_step) {
      continue;
    }
    Vec2d prev_vec(prev_point.path_point().x(), prev_point.path_point().y());
    Vec2d curr_vec(curr_point.path_point().x(), curr_point.path_point().y());
    segments.emplace_back(prev_vec, curr_vec);

    prev_point = curr_point;
  }
  return segments;
}

bool ADCTrajectoryContainer::IsProtected() const {
  return adc_trajectory_.has_right_of_way_status() &&
         adc_trajectory_.right_of_way_status() == ADCTrajectory::PROTECTED;
}

}  // namespace prediction
}  // namespace apollo
