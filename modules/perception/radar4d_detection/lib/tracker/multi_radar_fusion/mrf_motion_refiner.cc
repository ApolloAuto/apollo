/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/radar4d_detection/lib/tracker/multi_radar_fusion/mrf_motion_refiner.h"

#include <algorithm>
#include <limits>

#include "cyber/common/file.h"
#include "modules/perception/common/util.h"
#include "modules/perception/common/algorithm/geometry/basic.h"

namespace apollo {
namespace perception {
namespace radar4d {

bool MrfMotionRefiner::Init(const MrfMotionRefinerInitOptions& options) {
  std::string config_file = "mrf_motion_refiner.pb.txt";
  if (!options.config_file.empty()) {
    config_file = options.config_file;
  }
  config_file = GetConfigFile(options.config_path, config_file);
  MrfMotionRefinerConfig config;
  ACHECK(apollo::cyber::common::GetProtoFromFile(config_file, &config));
  // read from proto config
  claping_speed_threshold_ = config.claping_speed_threshold();
  claping_acceleration_threshold_ = config.claping_acceleration_threshold();
  return true;
}

bool MrfMotionRefiner::Refine(const MrfTrackDataConstPtr& track_data,
                              TrackedObjectPtr new_object) {
  auto latest_object_pair = track_data->GetLatestObject();
  const TrackedObjectConstPtr& latest_object = latest_object_pair.second;

  // 1. keep motion if acceleration of filter is greater than a threshold
  double time_diff = new_object->object_ptr->latest_tracked_time -
                     latest_object->object_ptr->latest_tracked_time;
  Eigen::Vector3d filter_velocity_gain = new_object->belief_velocity_gain;
  double filter_acceleration_gain = 0.0;
  if (fabs(time_diff) < EPSION_TIME) {
    filter_acceleration_gain = 0.0;
  } else {
    filter_acceleration_gain = filter_velocity_gain.norm() / time_diff;
  }
  bool need_keep_motion =
      filter_acceleration_gain > claping_acceleration_threshold_;
  // use tighter threshold for pedestrian
  if (filter_acceleration_gain > claping_acceleration_threshold_ / 2 &&
      new_object->type == base::ObjectType::PEDESTRIAN) {
    need_keep_motion = true;
  }

  if (need_keep_motion && (!new_object->converged)) {
    Eigen::Vector3d current_velocity = Eigen::Vector3d::Zero();
    current_velocity = latest_object->output_velocity;
    new_object->output_velocity = current_velocity;
    AINFO << "Track_id " << track_data->track_id_
          << ", keep motion because of extraodinary acceleration.";
    return true;
  }
  // 2. static hypothesis check
  bool is_static_hypothesis =
      CheckStaticHypothesisByState(latest_object, new_object);

  if (is_static_hypothesis) {
    new_object->output_velocity = Eigen::Vector3d::Zero();
    new_object->state.tail<2>().setZero();
    AINFO << "Track_id " << track_data->track_id_
          << ", set velocity to zero because of noise claping.";
    return true;
  }
  return false;
}

bool MrfMotionRefiner::CheckStaticHypothesisByState(
    const TrackedObjectConstPtr& latest_object,
    const TrackedObjectConstPtr& new_object) const {
  // Check whether track is static or not
  // evaluate speed noise level, the less the level is the
  // greater the probability of noise is
  double speed = new_object->output_velocity.head(2).norm();
  bool velocity_noise_level_is_0 = speed < (claping_speed_threshold_ / 8);
  bool velocity_noise_level_is_1 = speed < (claping_speed_threshold_ / 4);
  bool velocity_noise_level_is_2 = speed < (claping_speed_threshold_ / 2);
  bool velocity_noise_level_is_3 = speed < (claping_speed_threshold_ / 1);
  // believe track is staic if velocity noise level is 0
  // use loose threshold for object is not pedestrian
  if (velocity_noise_level_is_1 &&
      latest_object->type != base::ObjectType::PEDESTRIAN) {
    velocity_noise_level_is_0 = true;
  }
  if (velocity_noise_level_is_0) {
    return true;
  }
  // believe track is static if velocity noise level is
  // 1 && angle change level is 0
  // use loose threshold for object is not pedstrian
  if (velocity_noise_level_is_2 &&
      latest_object->type != base::ObjectType::PEDESTRIAN) {
    velocity_noise_level_is_1 = true;
  }
  double reasonable_angle_change_maximum_0 = M_PI / 6;
  bool velocity_angle_change_level_is_0 =
      CheckStaticHypothesisByVelocityAngleChange(
          latest_object, new_object, reasonable_angle_change_maximum_0);
  if (velocity_noise_level_is_1 && velocity_angle_change_level_is_0) {
    return true;
  }
  // believe track is static if velocity noise level is
  // 2 && angle change level is 1
  // use loose threshold for object is not pedestrian
  if (velocity_noise_level_is_3 &&
      latest_object->type != base::ObjectType::PEDESTRIAN) {
    velocity_noise_level_is_2 = true;
  }
  double reasonable_angle_change_maximum_1 = M_PI / 4;
  bool velocity_angle_change_level_is_1 =
      CheckStaticHypothesisByVelocityAngleChange(
          latest_object, new_object, reasonable_angle_change_maximum_1);
  if (velocity_noise_level_is_2 && velocity_angle_change_level_is_1) {
    return true;
  }
  return false;
}

bool MrfMotionRefiner::CheckStaticHypothesisByVelocityAngleChange(
    const TrackedObjectConstPtr& latest_object,
    const TrackedObjectConstPtr& new_object,
    double reasonable_angle_change_maximum) const {
  // note reasonable_angle_change_maximum should be within [0, M_PI]
  // believe angle change is obvious if one of estimation pair is
  // extrodinary small
  Eigen::Vector3d previous_velocity = latest_object->output_velocity;
  Eigen::Vector3d current_velocity = new_object->output_velocity;
  constexpr double kEpsilon = std::numeric_limits<double>::epsilon();
  if (previous_velocity.norm() < kEpsilon) {
    // return false; // without motion score, this should be true
    return true;
  }
  if (current_velocity.norm() < kEpsilon) {
    return true;
  }
  // believe angle change is obvious if it is greater than threshold

  double velocity_angle_change =
      algorithm::CalculateTheta2DXY(previous_velocity, current_velocity);
  if (fabs(velocity_angle_change) > reasonable_angle_change_maximum) {
    return true;
  }
  // Let track be static if angle change of current velocity
  // & previous heading is larger
  // than given reasonable maximum.
  Eigen::Vector3d previous_direction = latest_object->output_direction;
  double velocity_heading_angle_change_1 =
      algorithm::CalculateTheta2DXY(previous_direction, current_velocity);
  previous_direction *= -1;
  double velocity_heading_angle_change_2 =
      algorithm::CalculateTheta2DXY<double>(
          previous_direction, current_velocity);
  velocity_angle_change = std::min(fabs(velocity_heading_angle_change_1),
                                   fabs(velocity_heading_angle_change_2));
  if (fabs(velocity_angle_change) > reasonable_angle_change_maximum) {
    return true;
  }
  return false;
}

}  // namespace radar4d
}  // namespace perception
}  // namespace apollo
