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

#include "modules/perception/lidar_tracking/tracker/multi_lidar_fusion/mlf_motion_refiner.h"

#include <algorithm>
#include <limits>

#include "cyber/common/file.h"
#include "modules/perception/common/util.h"
#include "modules/perception/common/algorithm/geometry/basic.h"

namespace apollo {
namespace perception {
namespace lidar {

bool MlfMotionRefiner::Init(const MlfMotionRefinerInitOptions& options) {
  std::string config_file = "mlf_motion_refiner.conf";
  if (!options.config_file.empty()) {
    config_file = options.config_file;
  }
  config_file = GetConfigFile(options.config_path, config_file);
  MlfMotionRefinerConfig config;
  ACHECK(apollo::cyber::common::GetProtoFromFile(config_file, &config));
  // read from proto config
  claping_speed_threshold_ = config.claping_speed_threshold();
  claping_acceleration_threshold_ = config.claping_acceleration_threshold();
  cyc_refine_speed_ = config.cyc_refine_speed();
  car_refine_speed_ = config.car_refine_speed();
  use_movable_state_check_ = config.use_movable_state_check();
  return true;
}

bool MlfMotionRefiner::Refine(const MlfTrackDataConstPtr& track_data,
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
    // new_object->output_velocity = Eigen::Vector3d::Zero();
    // new_object->state.tail<2>().setZero();
    // new_object->belief_velocity = Eigen::Vector3d::Zero();
    // new_object->state.head<2>().setZero();
    AINFO << "[Velocity-Refine] track_id: " << track_data->track_id_
          << " KEEP MOTION because of extraodinary acceleration.";
    return true;
  }

  // 2. motion state check
  bool need_assign_movable_obj_speed =
      NeedAssignMovableObjectSpeed(latest_object, new_object);
  if (use_movable_state_check_ && need_assign_movable_obj_speed) {
    if (new_object->belief_velocity.head(2).norm() > 0) {
      new_object->output_velocity = new_object->belief_velocity;
    } else if (new_object->measured_barycenter_velocity.head(2).norm() > 0) {
      new_object->output_velocity = new_object->measured_barycenter_velocity;
    } else {
      new_object->output_velocity = new_object->measured_center_velocity;
    }
    return true;
  }

  // 3. static hypothesis check
  bool is_static_hypothesis =
      CheckStaticHypothesisByState(latest_object, new_object);

  if (is_static_hypothesis) {
    new_object->output_velocity = Eigen::Vector3d::Zero();
    new_object->state.tail<2>().setZero();
    AINFO << "[Velocity-Refine] track_id: " << track_data->track_id_
          << " set velocity to zero because of static_hypothesis.";
    return true;
  }
  return false;
}

bool MlfMotionRefiner::CheckStaticHypothesisByState(
    const TrackedObjectConstPtr& latest_object,
    const TrackedObjectConstPtr& new_object) const {
  // Check whether track is static or not
    // evaluate speed noise level, the less the level is the
    // greater the probability of noise is
    double speed = new_object->output_velocity.head(2).norm();

    // TypeBasedThreshold
    bool velocity_noise_level_0 = false;
    bool velocity_noise_level_1 = false;
    bool velocity_noise_level_2 = false;
    auto type_select = new_object->type;
    // for pedestrian we give loose threshold
    if (type_select == base::ObjectType::PEDESTRIAN ||
        type_select == base::ObjectType::UNKNOWN) {
        velocity_noise_level_0 = speed < (claping_speed_threshold_ / 4.0);
        velocity_noise_level_1 = speed < (claping_speed_threshold_ / 2.0);
        velocity_noise_level_2 = speed < (claping_speed_threshold_ / 1.0);
    } else if (type_select == base::ObjectType::BICYCLE) {
        velocity_noise_level_0 = speed < (cyc_refine_speed_ / 6.0);
        velocity_noise_level_1 = speed < (cyc_refine_speed_ / 4.0);
        velocity_noise_level_2 = speed < (cyc_refine_speed_ / 2.0);
    } else if (type_select == base::ObjectType::VEHICLE) {
        velocity_noise_level_0 = speed < (car_refine_speed_ / 8.0);
        velocity_noise_level_1 = speed < (car_refine_speed_ / 4.0);
        velocity_noise_level_2 = speed < (car_refine_speed_ / 2.0);
    }

    // believe track is static if speed_level 0 is so small
    if (velocity_noise_level_0) {
        ADEBUG << "type: " << static_cast<size_t>(type_select)
               << " speed is " << speed << " so small";
        return true;
    }

    double velocity_change_angle = M_PI, velocity_heading_angle = M_PI;
    if (CalculateVelocityAngleChange(latest_object, new_object,
        &velocity_change_angle, &velocity_heading_angle)) {
        ADEBUG << " velocity_change_angle: " << velocity_change_angle
               << " velocity_heading_angle: " << velocity_heading_angle;
    }

    // believe track is static
    // speed_level 1 and velocity-change or velocity-heading change > PI/6
    bool dir_change_level_0 = (fabs(velocity_change_angle) > M_PI / 6 ||
          fabs(velocity_heading_angle) > M_PI / 6);
    if (velocity_noise_level_1 && dir_change_level_0) {
        ADEBUG << "type: " << static_cast<size_t>(type_select) << " speed is "
               << speed << " velocity_change is " << velocity_change_angle
               << " velocity_heading is " << velocity_heading_angle
               << " bigger than PI/6";
        return true;
    }

    // believe track is static
    // speed_level 2 and velocity-change or velocity-heading change is > PI/4
    bool dir_change_level_1 = (fabs(velocity_change_angle) > M_PI / 4 ||
          fabs(velocity_heading_angle) > M_PI / 4);
    if (velocity_noise_level_2 && dir_change_level_1) {
        ADEBUG << "type: " << static_cast<size_t>(type_select) << " speed is "
               << speed << " velocity_change is " << velocity_change_angle
               << " velocity_heading is " << velocity_heading_angle
               << " bigger than PI/4";
        return true;
    }
    return false;
}

bool MlfMotionRefiner::CalculateVelocityAngleChange(
        const TrackedObjectConstPtr& latest_object,
        const TrackedObjectConstPtr& new_object,
        double* vel_change_angle,
        double* velocity_heading_angle) const {
    Eigen::Vector3d pre_velocity = latest_object->output_velocity;
    Eigen::Vector3d cur_velocity = new_object->output_velocity;
    Eigen::Vector3d pre_direction = latest_object->output_direction;
    constexpr double kEpsilon = std::numeric_limits<double>::epsilon();

    if (pre_velocity.norm() < kEpsilon || cur_velocity.norm() < kEpsilon) {
        return false;
    }
    *vel_change_angle = algorithm::CalculateTheta2DXY<double>(
        pre_velocity, cur_velocity);

    double velocity_heading_angle_1 = algorithm::CalculateTheta2DXY<double>(
        pre_direction, cur_velocity);
    pre_direction *= -1;
    double velocity_heading_angle_2 = algorithm::CalculateTheta2DXY<double>(
        pre_direction, cur_velocity);
    *velocity_heading_angle = std::min(fabs(velocity_heading_angle_1),
        fabs(velocity_heading_angle_2));
    return true;
}

bool MlfMotionRefiner::CheckStaticHypothesisByVelocityAngleChange(
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

bool MlfMotionRefiner::NeedAssignMovableObjectSpeed(
    const TrackedObjectConstPtr& latest_object,
    const TrackedObjectConstPtr& new_object) const {
  base::ObjectPtr obj = new_object->object_ptr;
  // moving object and have no speed
  if (obj->motion_state == base::MotionState::MOVING &&
      new_object->output_velocity.head(2).norm() < 1e-6) {
    return true;
  }

  return false;
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
