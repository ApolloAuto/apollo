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
#include "modules/perception/obstacle/radar/modest/object_builder.h"

#include <cmath>
// #include "modules/perception/common/perception_gflags.h"
#include "modules/perception/obstacle/radar/modest/conti_radar_util.h"
#include "modules/perception/obstacle/radar/modest/radar_util.h"

namespace apollo {
namespace perception {

bool ObjectBuilder::Build(const ContiRadar &raw_obstacles,
                          const Eigen::Matrix4d &radar_pose,
                          const Eigen::Vector2d &main_velocity,
                          SensorObjects *radar_objects) {
  if (radar_objects == nullptr) {
    AERROR << "radar objects is nullptr.";
    return false;
  }
  std::unordered_map<int, int> current_con_ids;
  auto objects = &(radar_objects->objects);
  for (int i = 0; i < raw_obstacles.contiobs_size(); ++i) {
    std::shared_ptr<Object> object_ptr = std::shared_ptr<Object>(new Object());
    const int obstacle_id = raw_obstacles.contiobs(i).obstacle_id();
    auto continuous_id_it = continuous_ids_.find(obstacle_id);
    if (continuous_id_it != continuous_ids_.end()) {
      current_con_ids[obstacle_id] = continuous_id_it->second + 1;
    } else {
      current_con_ids[obstacle_id] = 1;
    }
    if (current_con_ids[obstacle_id] <= delay_frames_) {
      object_ptr->is_background = true;
    }
    int tracking_times = current_con_ids[obstacle_id];
    if (use_fp_filter_ &&
        ContiRadarUtil::IsFp(raw_obstacles.contiobs(i), conti_params_,
                             delay_frames_, tracking_times)) {
      object_ptr->is_background = true;
    }
    object_ptr->track_id = obstacle_id;
    Eigen::Matrix<double, 4, 1> location_r;
    Eigen::Matrix<double, 4, 1> location_w;
    location_r << raw_obstacles.contiobs(i).longitude_dist(),
        raw_obstacles.contiobs(i).lateral_dist(), 0.0, 1.0;
    location_w = radar_pose * location_r;
    Eigen::Vector3d point;
    point = location_w.topLeftCorner(3, 1);
    object_ptr->center = point;
    object_ptr->anchor_point = object_ptr->center;
    Eigen::Matrix<double, 3, 1> velocity_r;
    Eigen::Matrix<double, 3, 1> velocity_w;
    velocity_r << raw_obstacles.contiobs(i).longitude_vel(),
        raw_obstacles.contiobs(i).lateral_vel(), 0.0;
    velocity_w = radar_pose.topLeftCorner(3, 3) * velocity_r;

    //  calculate the absolute velodity
    object_ptr->velocity(0) = velocity_w[0] + main_velocity(0);
    object_ptr->velocity(1) = velocity_w[1] + main_velocity(1);
    object_ptr->velocity(2) = 0;

    Eigen::Vector3f ref_velocity(main_velocity(0), main_velocity(1), 0.0);
    if (ContiRadarUtil::IsConflict(ref_velocity,
                                   object_ptr->velocity.cast<float>())) {
      object_ptr->is_background = true;
    }

    object_ptr->length = 1.0;
    object_ptr->width = 1.0;
    object_ptr->height = 1.0;
    object_ptr->type = ObjectType::UNKNOWN;
    object_ptr->score_type = ScoreType::SCORE_RADAR;
    object_ptr->score =
        static_cast<float>(raw_obstacles.contiobs(i).probexist());

    Eigen::Matrix3d dist_rms;
    Eigen::Matrix3d vel_rms;
    vel_rms.setZero();
    dist_rms.setZero();
    dist_rms(0, 0) = raw_obstacles.contiobs(i).longitude_dist_rms();
    dist_rms(1, 1) = raw_obstacles.contiobs(i).lateral_dist_rms();
    vel_rms(0, 0) = raw_obstacles.contiobs(i).longitude_vel_rms();
    vel_rms(1, 1) = raw_obstacles.contiobs(i).lateral_vel_rms();
    object_ptr->position_uncertainty =
        radar_pose.topLeftCorner(3, 3) * dist_rms * dist_rms.transpose() *
        radar_pose.topLeftCorner(3, 3).transpose();
    object_ptr->velocity_uncertainty =
        radar_pose.topLeftCorner(3, 3) * vel_rms * vel_rms.transpose() *
        radar_pose.topLeftCorner(3, 3).transpose();

    double local_theta =
        raw_obstacles.contiobs(i).oritation_angle() / 180.0 * M_PI;
    Eigen::Vector3f direction =
        Eigen::Vector3f(cos(local_theta), sin(local_theta), 0);
    direction = radar_pose.topLeftCorner(3, 3).cast<float>() * direction;
    object_ptr->direction = direction.cast<double>();
    //  the avg time diff is from manual
    object_ptr->tracking_time = current_con_ids[obstacle_id] * 0.074;
    double theta = std::atan2(direction(1), direction(0));
    object_ptr->theta = theta;
    //  For radar obstacle , the polygon is supposed to have
    //  four mock point: around obstacle center.
    RadarUtil::MockRadarPolygon(point, object_ptr->length, object_ptr->width,
                                theta, &(object_ptr->polygon));
    if (object_ptr->radar_supplement == nullptr) {
      object_ptr->radar_supplement.reset(new RadarSupplement());
    }
    object_ptr->radar_supplement->range = std::sqrt(
        location_r[0] * location_r[0] + location_r[1] * location_r[1]);

    object_ptr->radar_supplement->angle = 0;
    objects->push_back(object_ptr);
  }
  continuous_ids_ = current_con_ids;
  return true;
}

}  // namespace perception
}  // namespace apollo
