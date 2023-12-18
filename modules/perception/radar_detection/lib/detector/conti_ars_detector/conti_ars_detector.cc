/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "modules/perception/radar_detection/lib/detector/conti_ars_detector/conti_ars_detector.h"

#include "Eigen/Core"

#include "modules/perception/radar_detection/common/radar_util.h"
#include "modules/perception/radar_detection/common/types.h"

namespace apollo {
namespace perception {
namespace radar {

bool ContiArsDetector::Init(const DetectorInitOptions& options) { return true; }

bool ContiArsDetector::Detect(const drivers::ContiRadar& corrected_obstacles,
                              const DetectorOptions& options,
                              base::FramePtr radar_frame) {
  RawObs2Frame(corrected_obstacles, options, radar_frame);
  radar_frame->timestamp = corrected_obstacles.header().timestamp_sec();
  radar_frame->sensor2world_pose = *(options.radar2world_pose);
  return true;
}

void ContiArsDetector::RawObs2Frame(
    const drivers::ContiRadar& corrected_obstacles,
    const DetectorOptions& options, base::FramePtr radar_frame) {
  const Eigen::Matrix4d& radar2world = *(options.radar2world_pose);
  const Eigen::Matrix4d& radar2novatel = *(options.radar2novatel_trans);
  const Eigen::Vector3f& angular_speed = options.car_angular_speed;
  Eigen::Matrix3d rotation_novatel;
  rotation_novatel << 0, -angular_speed(2), angular_speed(1), angular_speed(2),
      0, -angular_speed(0), -angular_speed(1), angular_speed(0), 0;
  Eigen::Matrix3d rotation_radar = radar2novatel.topLeftCorner(3, 3).inverse() *
                                   rotation_novatel *
                                   radar2novatel.topLeftCorner(3, 3);
  Eigen::Matrix3d radar2world_rotate = radar2world.block<3, 3>(0, 0);
  Eigen::Matrix3d radar2world_rotate_t = radar2world_rotate.transpose();
  // Eigen::Vector3d radar2world_translation = radar2world.block<3, 1>(0, 3);
  ADEBUG << "radar2novatel: " << radar2novatel;
  ADEBUG << "angular_speed: " << angular_speed;
  ADEBUG << "rotation_radar: " << rotation_radar;
  for (const auto radar_obs : corrected_obstacles.contiobs()) {
    base::ObjectPtr radar_object(new base::Object);
    radar_object->id = radar_obs.obstacle_id();
    radar_object->track_id = radar_obs.obstacle_id();
    Eigen::Vector4d local_loc(radar_obs.longitude_dist(),
                              radar_obs.lateral_dist(), 0, 1);
    Eigen::Vector4d world_loc =
        static_cast<Eigen::Matrix<double, 4, 1, 0, 4, 1>>(radar2world *
                                                          local_loc);
    radar_object->center = world_loc.block<3, 1>(0, 0);
    radar_object->anchor_point = radar_object->center;

    Eigen::Vector3d local_vel(radar_obs.longitude_vel(),
                              radar_obs.lateral_vel(), 0);

    Eigen::Vector3d angular_trans_speed =
        rotation_radar * local_loc.topLeftCorner(3, 1);
    Eigen::Vector3d world_vel =
        static_cast<Eigen::Matrix<double, 3, 1, 0, 3, 1>>(
            radar2world_rotate * (local_vel + angular_trans_speed));
    Eigen::Vector3d vel_temp =
        world_vel + options.car_linear_speed.cast<double>();
    radar_object->velocity = vel_temp.cast<float>();

    Eigen::Matrix3d dist_rms;
    dist_rms.setZero();
    Eigen::Matrix3d vel_rms;
    vel_rms.setZero();
    dist_rms(0, 0) = radar_obs.longitude_dist_rms();
    dist_rms(1, 1) = radar_obs.lateral_dist_rms();
    vel_rms(0, 0) = radar_obs.longitude_vel_rms();
    vel_rms(1, 1) = radar_obs.lateral_vel_rms();
    radar_object->center_uncertainty =
        (radar2world_rotate * dist_rms * dist_rms.transpose() *
         radar2world_rotate_t)
            .cast<float>();

    radar_object->velocity_uncertainty =
        (radar2world_rotate * vel_rms * vel_rms.transpose() *
         radar2world_rotate_t)
            .cast<float>();
    double local_obj_theta = radar_obs.oritation_angle() / 180.0 * PI;
    Eigen::Vector3f direction(static_cast<float>(cos(local_obj_theta)),
                              static_cast<float>(sin(local_obj_theta)), 0.0f);
    direction = radar2world_rotate.cast<float>() * direction;
    radar_object->direction = direction;
    radar_object->theta = std::atan2(direction(1), direction(0));
    radar_object->theta_variance =
        static_cast<float>(radar_obs.oritation_angle_rms() / 180.0 * PI);
    radar_object->confidence = static_cast<float>(radar_obs.probexist());

    int motion_state = radar_obs.dynprop();
    double prob_target = radar_obs.probexist();
    if ((prob_target > MIN_PROBEXIST) &&
        (motion_state == CONTI_MOVING || motion_state == CONTI_ONCOMING ||
         motion_state == CONTI_CROSSING_MOVING)) {
      radar_object->motion_state = base::MotionState::MOVING;
    } else if (motion_state == CONTI_DYNAMIC_UNKNOWN) {
      radar_object->motion_state = base::MotionState::UNKNOWN;
    } else {
      radar_object->motion_state = base::MotionState::STATIONARY;
      radar_object->velocity.setZero();
    }

    int cls = radar_obs.obstacle_class();
    if (cls == CONTI_CAR || cls == CONTI_TRUCK) {
      radar_object->type = base::ObjectType::VEHICLE;
    } else if (cls == CONTI_PEDESTRIAN) {
      radar_object->type = base::ObjectType::PEDESTRIAN;
    } else if (cls == CONTI_MOTOCYCLE || cls == CONTI_BICYCLE) {
      radar_object->type = base::ObjectType::BICYCLE;
    } else {
      radar_object->type = base::ObjectType::UNKNOWN;
    }

    radar_object->size(0) = static_cast<float>(radar_obs.length());
    radar_object->size(1) = static_cast<float>(radar_obs.width());
    radar_object->size(2) = 2.0f;  // vehicle template (pnc required)
    if (cls == CONTI_POINT) {
      radar_object->size(0) = 1.0f;
      radar_object->size(1) = 1.0f;
    }
    // extreme case protection
    if (radar_object->size(0) * radar_object->size(1) < 1.0e-4) {
      if (cls == CONTI_CAR || cls == CONTI_TRUCK) {
        radar_object->size(0) = 4.0f;
        radar_object->size(1) = 1.6f;  // vehicle template
      } else {
        radar_object->size(0) = 1.0f;
        radar_object->size(1) = 1.0f;
      }
    }
    MockRadarPolygon(radar_object);

    float local_range = static_cast<float>(local_loc.head(2).norm());
    float local_angle =
        static_cast<float>(std::atan2(local_loc(1), local_loc(0)));
    radar_object->radar_supplement.range = local_range;
    radar_object->radar_supplement.angle = local_angle;

    radar_frame->objects.push_back(radar_object);

    ADEBUG << "obs_id: " << radar_obs.obstacle_id() << ", "
           << "long_dist: " << radar_obs.longitude_dist() << ", "
           << "lateral_dist: " << radar_obs.lateral_dist() << ", "
           << "long_vel: " << radar_obs.longitude_vel() << ", "
           << "latera_vel: " << radar_obs.lateral_vel() << ", "
           << "rcs: " << radar_obs.rcs() << ", "
           << "meas_state: " << radar_obs.meas_state();
  }
}

PERCEPTION_REGISTER_DETECTOR(ContiArsDetector);

}  // namespace radar
}  // namespace perception
}  // namespace apollo
