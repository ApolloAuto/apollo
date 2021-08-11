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

#include <algorithm>
#include <string>
#include <vector>

#include "modules/perception/lidar/lib/tracker/measurement/measurement_collection.h"
#include "modules/perception/lidar/lib/tracker/multi_lidar_fusion/mlf_motion_measurement.h"

namespace apollo {
namespace perception {
namespace lidar {

void MlfMotionMeasurement::ComputeMotionMeasurment(
    const MlfTrackDataConstPtr& track_data, TrackedObjectPtr new_object) {
  // prefer to choose objects from the same sensor
  std::string sensor_name = new_object->sensor_info.name;
  TrackedObjectConstPtr latest_object =
      track_data->GetLatestSensorObject(sensor_name).second;
  if (latest_object == nullptr) {
    latest_object = track_data->GetLatestObject().second;
  }
  if (latest_object.get() == nullptr) {
    AERROR << "latest_object is not available";
    return;
  }
  // should we estimate the measurement if the time diff is too small?
  double latest_time = latest_object->object_ptr->latest_tracked_time;
  double current_time = new_object->object_ptr->latest_tracked_time;
  double time_diff = current_time - latest_time;
  if (fabs(time_diff) < EPSILON_TIME) {
    time_diff = DEFAULT_FPS;
  }
  MeasureAnchorPointVelocity(new_object, latest_object, time_diff);
  MeasureBboxCenterVelocity(new_object, latest_object, time_diff);
  MeasureBboxCornerVelocity(new_object, latest_object, time_diff);
  MeasurementSelection(track_data, latest_object, new_object);
  MeasurementQualityEstimation(latest_object, new_object);
}

void MlfMotionMeasurement::MeasurementSelection(
    const MlfTrackDataConstPtr& track_data,
    const TrackedObjectConstPtr& latest_object, TrackedObjectPtr new_object) {
  // Select measured velocity among candidates according motion consistency
  int64_t corner_index = 0;
  float corner_velocity_gain = 0.0f;
  std::vector<float> corner_velocity_gain_norms(4);
  for (int i = 0; i < 4; ++i) {
    corner_velocity_gain_norms[i] =
        static_cast<float>((new_object->measured_corners_velocity[i] -
                            latest_object->belief_velocity)
                               .norm());
  }
  std::vector<float>::iterator corener_min_gain =
      std::min_element(std::begin(corner_velocity_gain_norms),
                       std::end(corner_velocity_gain_norms));
  corner_velocity_gain = *corener_min_gain;
  corner_index = corener_min_gain - corner_velocity_gain_norms.begin();

  std::vector<float> velocity_gain_norms(3);
  velocity_gain_norms[0] = corner_velocity_gain;
  velocity_gain_norms[1] =
      static_cast<float>((new_object->measured_barycenter_velocity -
                          latest_object->belief_velocity)
                             .norm());
  velocity_gain_norms[2] = static_cast<float>(
      (new_object->measured_center_velocity - latest_object->belief_velocity)
          .norm());
  std::vector<float>::iterator min_gain = std::min_element(
      std::begin(velocity_gain_norms), std::end(velocity_gain_norms));
  int64_t min_gain_pos = min_gain - velocity_gain_norms.begin();
  if (min_gain_pos == 0) {
    new_object->selected_measured_velocity =
        new_object->measured_corners_velocity[corner_index];
  }
  if (min_gain_pos == 1) {
    new_object->selected_measured_velocity =
        new_object->measured_barycenter_velocity;
  }
  if (min_gain_pos == 2) {
    new_object->selected_measured_velocity =
        new_object->measured_center_velocity;
  }
}

void MlfMotionMeasurement::MeasurementQualityEstimation(
    const TrackedObjectConstPtr& latest_object, TrackedObjectPtr new_object) {
  // 1. point size diff (only for same sensor)
  int pre_num = static_cast<int>(
      latest_object->object_ptr->lidar_supplement.cloud_world.size());
  int cur_num = static_cast<int>(
      new_object->object_ptr->lidar_supplement.cloud_world.size());
  double quality_based_on_point_diff_ratio =
      (latest_object->sensor_info.name == new_object->sensor_info.name)
          ? (1.0 - fabs(pre_num - cur_num) / std::max(pre_num, cur_num))
          : 1.0;
  // 2. association quality
  double quality_based_on_association_score =
      pow(1.0 - new_object->association_score, 2.0);
  new_object->update_quality = std::min(quality_based_on_association_score,
                                        quality_based_on_point_diff_ratio);
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
