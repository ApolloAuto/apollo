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
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "Eigen/Core"

#include "modules/perception/base/object.h"
#include "modules/perception/base/object_pool_types.h"
#include "modules/perception/base/object_types.h"

namespace apollo {
namespace perception {
namespace lidar {

struct TrackedObject {
  TrackedObject() = default;
  TrackedObject(base::ObjectPtr obj_ptr, const Eigen::Affine3d& pose);
  TrackedObject(const TrackedObject& rhs) = default;
  TrackedObject& operator=(const TrackedObject& rhs) = default;

  void Reset();

  void Reset(
      base::ObjectPtr obj_ptr, const Eigen::Affine3d& pose,
      const Eigen::Vector3d& global_to_local_offset = Eigen::Vector3d::Zero(),
      const base::SensorInfo& sensor = base::SensorInfo());

  std::string ToString() const;
  void ComputeShapeFeatures();

  void AttachObject(
      base::ObjectPtr obj_ptr, const Eigen::Affine3d& pose,
      const Eigen::Vector3d& global_to_local_offset = Eigen::Vector3d::Zero(),
      const base::SensorInfo& sensor = base::SensorInfo());

  void TransformObjectCloudToWorld();

  void CopyFrom(std::shared_ptr<TrackedObject> rhs, bool is_deep);

  void ToObject(base::ObjectPtr obj_ptr) const;

  float GetVelThreshold(base::ObjectPtr obj) const;
  // ***************************************************
  // self information from match
  // ***************************************************
  std::vector<float> shape_features;
  std::vector<float> shape_features_full;
  size_t histogram_bin_size = 10;
  // association distance
  // range from 0 to max_match_distance
  float association_score = 0.0f;

  // ***************************************************
  // self information from track
  // ***************************************************
  bool is_fake = false;
  int track_id = -1;
  double tracking_time = 0.0;

  // ***************************************************
  // information from main car
  // ***************************************************
  Eigen::Affine3d sensor_to_local_pose;

  // ***************************************************
  // measurement correlative information from object_ptr
  // ***************************************************
  base::ObjectPtr object_ptr;
  // corners always store follow const order based on object direction
  Eigen::Vector3d corners[4];
  Eigen::Vector3d center;
  Eigen::Vector3d barycenter;
  Eigen::Vector3d anchor_point;

  // oriented
  Eigen::Vector3d direction;
  Eigen::Vector3d lane_direction;
  Eigen::Vector3d size;

  base::ObjectType type = base::ObjectType::UNKNOWN;
  bool is_background = false;

  // ***************************************************
  // measurement correlative information from measurement computer
  // ***************************************************
  Eigen::Vector3d measured_barycenter_velocity;
  Eigen::Vector3d measured_center_velocity;
  Eigen::Vector3d measured_nearest_corner_velocity;  // no projection
  Eigen::Vector3d measured_corners_velocity[4];

  // ***************************************************
  // filter correlative information
  // ***************************************************
  // states
  int boostup_need_history_size = 0;
  bool valid = false;
  bool converged = true;
  float convergence_confidence = 0.0f;
  double update_quality = 0.0;
  Eigen::Vector3d selected_measured_velocity;
  Eigen::Vector3d selected_measured_acceleration;
  Eigen::Vector3d belief_anchor_point;
  Eigen::Vector3d belief_velocity;
  Eigen::Vector3d belief_acceleration;
  Eigen::Vector3d belief_velocity_gain;

  // filter covariances
  Eigen::Matrix3d velocity_covariance;
  Eigen::Matrix3d belief_velocity_online_covariance;

  // combine velocity and acceleration
  Eigen::Vector4d state;
  Eigen::Matrix4d measurement_covariance;
  Eigen::Matrix4d state_covariance;

  // motion score (calculated in kalman_filter)
  // the three scores are
  // (smaller score means high probability of motion state):
  // 1. average value of (variance of velocity norm / average velocity norm)
  // 2. average value of abs(theta diff)
  // 3. variance of score1
  Eigen::Vector3d motion_score;

  // ***************************************************
  // postprocess correlative information
  // ***************************************************
  Eigen::Vector3d output_velocity;
  Eigen::Matrix3d output_velocity_uncertainty;
  Eigen::Vector3d output_direction;
  Eigen::Vector3d output_center;
  Eigen::Vector3d output_size;
  base::SensorInfo sensor_info;

  Eigen::Vector3d global_local_offset;
};  // struct TrackedObject

typedef std::shared_ptr<TrackedObject> TrackedObjectPtr;
typedef std::shared_ptr<const TrackedObject> TrackedObjectConstPtr;

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
