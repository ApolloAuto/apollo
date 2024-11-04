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

#include "modules/perception/common/base/object_supplement.h"
#include "modules/perception/common/base/object_types.h"
#include "modules/perception/common/base/point_cloud.h"
#include "modules/perception/common/base/vehicle_struct.h"
// #include "modules/common_msgs/prediction_msgs/feature.pb.h"

namespace apollo {
namespace perception {
namespace base {

struct Object {
  Object();
  std::string ToString() const;
  void Reset();

  // @brief object id per frame, required
  int id = -1;

  // @brief convex hull of the object, required
  PointCloud<PointD> polygon;

  // oriented boundingbox information
  // @brief main direction of the object, required
  Eigen::Vector3f direction = Eigen::Vector3f(1, 0, 0);
  /*@brief the yaw angle, theta = 0.0 <=> direction(1, 0, 0),
    currently roll and pitch are not considered,
    make sure direction and theta are consistent, required
  */
  float theta = 0.0f;
  // @brief theta variance, required
  float theta_variance = 0.0f;
  // @brief center of the boundingbox (cx, cy, cz), required
  Eigen::Vector3d center = Eigen::Vector3d(0, 0, 0);
  // @brief covariance matrix of the center uncertainty, required
  Eigen::Matrix3f center_uncertainty;
  /* @brief size = [length, width, height] of boundingbox
     length is the size of the main direction, required
  */
  Eigen::Vector3f size = Eigen::Vector3f(0, 0, 0);
  // @brief size variance, required
  Eigen::Vector3f size_variance = Eigen::Vector3f(0, 0, 0);
  // @brief anchor point, required
  Eigen::Vector3d anchor_point = Eigen::Vector3d(0, 0, 0);

  // @brief object type, required
  ObjectType type = ObjectType::UNKNOWN;
  // @brief probability for each type, required
  std::vector<float> type_probs;

  // @brief object sub-type, optional
  ObjectSubType sub_type = ObjectSubType::UNKNOWN;
  // @brief probability for each sub-type, optional
  std::vector<float> sub_type_probs;

  // @brief existence confidence, required
  float confidence = 1.0f;

  // @brief: attention on car-front-critical objects
  bool is_front_critical = false;

  // tracking information
  // @brief track id, required
  int track_id = -1;
  // @brief velocity of the object, required
  Eigen::Vector3f velocity = Eigen::Vector3f(0, 0, 0);
  // @brief covariance matrix of the velocity uncertainty, required
  Eigen::Matrix3f velocity_uncertainty;
  // @brief if the velocity estimation is converged, true by default
  bool velocity_converged = true;
  // @brief velocity confidence, required
  float velocity_confidence = 1.0f;
  // @brief acceleration of the object, required
  Eigen::Vector3f acceleration = Eigen::Vector3f(0, 0, 0);
  // @brief covariance matrix of the acceleration uncertainty, required
  Eigen::Matrix3f acceleration_uncertainty;

  // @brief age of the tracked object, required
  double tracking_time = 0.0;
  // @brief timestamp of latest measurement, required
  double latest_tracked_time = 0.0;

  // @brief motion state of the tracked object, required
  MotionState motion_state = MotionState::UNKNOWN;
  // Tailgating (trajectory of objects)
  std::array<Eigen::Vector3d, 100> drops;
  std::size_t drop_num = 0;
  // CIPV
  bool b_cipv = false;
  // @brief brake light, left-turn light and right-turn light score, optional
  CarLight car_light;
  // @brief sensor-specific object supplements, optional
  LidarObjectSupplement lidar_supplement;
  Radar4dObjectSupplement radar4d_supplement;
  RadarObjectSupplement radar_supplement;
  CameraObjectSupplement camera_supplement;
  FusionObjectSupplement fusion_supplement;

  // @debug feature to be used for semantic mapping
//  std::shared_ptr<apollo::prediction::Feature> feature;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

using ObjectPtr = std::shared_ptr<Object>;
using ObjectConstPtr = std::shared_ptr<const Object>;

}  // namespace base
}  // namespace perception
}  // namespace apollo
