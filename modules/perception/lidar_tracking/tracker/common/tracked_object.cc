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
#include "modules/perception/lidar_tracking/tracker/common/tracked_object.h"

#include "cyber/common/log.h"
#include "modules/perception/common/base/object_types.h"
#include "modules/perception/common/base/point.h"
#include "modules/perception/common/base/point_cloud.h"
#include "modules/perception/common/algorithm/point_cloud_processing/common.h"
#include "modules/perception/common/algorithm/geometry/common.h"
#include "modules/perception/common/lidar/common/feature_descriptor.h"

namespace apollo {
namespace perception {
namespace lidar {

using PointFCloud = apollo::perception::base::AttributePointCloud<base::PointF>;

TrackedObject::TrackedObject(base::ObjectPtr obj_ptr,
                             const Eigen::Affine3d& pose) {
  AttachObject(obj_ptr, pose);
}

void TrackedObject::AttachObject(base::ObjectPtr obj_ptr,
                                 const Eigen::Affine3d& pose,
                                 const Eigen::Vector3d& global_to_local_offset,
                                 const base::SensorInfo& sensor,
                                 double timestamp) {
  if (obj_ptr) {
    // all state of input obj_ptr will not change except cloud world
    object_ptr = obj_ptr;

    sensor_to_local_pose = pose;
    global_local_offset = global_to_local_offset;
    // object info to tracked object
    center = pose * object_ptr->center;
    const PointFCloud& cloud = (object_ptr->lidar_supplement).cloud;
    barycenter = (algorithm::CalculateCentroid(cloud)).cast<double>();
    barycenter = pose * barycenter;
    anchor_point = barycenter;

    // object-detection center
    float detection_center_x = obj_ptr->lidar_supplement.detections[0];
    float detection_center_y = obj_ptr->lidar_supplement.detections[1];
    float detection_center_z = obj_ptr->lidar_supplement.detections[2];
    float detection_size_x = obj_ptr->lidar_supplement.detections[3];
    float detection_size_y = obj_ptr->lidar_supplement.detections[4];
    float detection_size_z = obj_ptr->lidar_supplement.detections[5];
    float detection_theta = obj_ptr->lidar_supplement.detections[6];
    Eigen::Vector3d model_center_lidar = Eigen::Vector3d(detection_center_x,
        detection_center_y, detection_center_z);
    detection_center = pose * model_center_lidar;

    // object-detection center/size/direction -> corners
    Eigen::Matrix<float, 8, 1> corners;
    algorithm::CalculateCornersFromCenter(
        detection_center_x, detection_center_y, detection_center_z,
        detection_size_x, detection_size_y, detection_size_z,
        detection_theta, &corners);
    for (size_t i = 0; i < 4; i++) {
        Eigen::Vector3d detection_cor = Eigen::Vector3d(
            static_cast<double>(corners[2 * i]),
            static_cast<double>(corners[2 * i + 1]), detection_center_z);
        detection_corners[i] = pose * detection_cor;
    }

    Eigen::Matrix3d rotation = pose.rotation();
    direction = rotation * object_ptr->direction.cast<double>();
    lane_direction = direction;
    size = object_ptr->size.cast<double>();
    type = object_ptr->type;
    type_probs = object_ptr->type_probs;
    is_background = object_ptr->lidar_supplement.is_background;

    base::PointDCloud& cloud_world = (object_ptr->lidar_supplement).cloud_world;
    cloud_world.clear();
    cloud_world.resize(cloud.size());
    for (size_t i = 0; i < cloud.size(); ++i) {
      Eigen::Vector3d pt(cloud[i].x, cloud[i].y, cloud[i].z);
      Eigen::Vector3d pt_world = pose * pt;
      cloud_world[i].x = pt_world(0);
      cloud_world[i].y = pt_world(1);
      cloud_world[i].z = pt_world(2);
      cloud_world[i].intensity = cloud[i].intensity;
    }
    // memcpy(&(cloud_world.points_height(0)), &(cloud.points_height(0)),
    //        sizeof(float) * cloud.size());
    for (size_t i = 0; i < cloud.size(); ++i) {
      cloud_world.SetPointHeight(i, cloud.points_height(i));
    }
    // other belief information keep as Reset()
    selected_measured_velocity = Eigen::Vector3d::Zero();
    selected_measured_acceleration = Eigen::Vector3d::Zero();
    belief_anchor_point = barycenter;
    belief_velocity = Eigen::Vector3d::Zero();
    belief_acceleration = Eigen::Vector3d::Zero();

    direction_state = Eigen::Vector3d::Zero();
    direction_state_covariance = Eigen::Matrix3d::Zero();
    output_angular = Eigen::Vector3d::Zero();
    direction_convergence_confidence = 0.0;
    direction_converged = false;

    output_velocity = Eigen::Vector3d::Zero();
    output_velocity_uncertainty = Eigen::Matrix3d::Zero();
    output_direction = direction;
    output_center = center;
    output_size = size;

    sensor_info = sensor;
    this->timestamp = timestamp;
  }
}

void TrackedObject::TransformObjectCloudToWorld() {
  const base::PointFCloud& cloud = (object_ptr->lidar_supplement).cloud;
  base::PointDCloud& cloud_world = (object_ptr->lidar_supplement).cloud_world;
  cloud_world.clear();
  cloud_world.resize(cloud.size());
  for (size_t i = 0; i < cloud.size(); ++i) {
    Eigen::Vector3d pt(cloud[i].x, cloud[i].y, cloud[i].z);
    Eigen::Vector3d pt_world = sensor_to_local_pose * pt;
    cloud_world[i].x = pt_world(0);
    cloud_world[i].y = pt_world(1);
    cloud_world[i].z = pt_world(2);
    cloud_world[i].intensity = cloud[i].intensity;
  }
  memcpy(&cloud_world.points_height(0), &cloud.points_height(0),
         sizeof(float) * cloud.size());
}

void TrackedObject::Reset() {
  object_ptr.reset();
  sensor_to_local_pose = Eigen::Affine3d::Identity();

  // measurement reset
  for (int i = 0; i < 4; ++i) {
    corners[i] = Eigen::Vector3d::Zero();
    detection_corners[i] = Eigen::Vector3d::Zero();
    measured_corners_velocity[i] = Eigen::Vector3d::Zero();
    measured_history_corners_velocity[i] = Eigen::Vector3d::Zero();
    measured_detection_history_corners_velocity[i] = Eigen::Vector3d::Zero();
  }
  center = Eigen::Vector3d::Zero();
  barycenter = Eigen::Vector3d::Zero();
  anchor_point = Eigen::Vector3d::Zero();
  detection_center = Eigen::Vector3d::Zero();
  measured_barycenter_velocity = Eigen::Vector3d::Zero();
  measured_center_velocity = Eigen::Vector3d::Zero();
  measured_nearest_corner_velocity = Eigen::Vector3d::Zero();
  measured_history_center_velocity = Eigen::Vector3d::Zero();
  measured_detection_center_velocity = Eigen::Vector3d::Zero();
  measured_detection_history_center_velocity = Eigen::Vector3d::Zero();
  measured_big_velocity_age = 0;
  direction = Eigen::Vector3d::Zero();
  lane_direction = Eigen::Vector3d::Zero();
  size = Eigen::Vector3d::Zero();
  type = base::ObjectType::UNKNOWN;
  type_probs.assign(static_cast<int>(base::ObjectType::MAX_OBJECT_TYPE), 0.0f);
  is_background = false;
  shape_features.clear();
  shape_features_full.clear();
  histogram_bin_size = 10;
  association_score = 0.0;
  is_fake = false;
  track_id = -1;
  tracking_time = 0.0;

  // filter reset
  boostup_need_history_size = 0;
  valid = false;
  converged = true;
  convergence_confidence = 0.0;
  update_quality = 0.0;
  selected_measured_velocity = measured_center_velocity;
  selected_measured_acceleration = Eigen::Vector3d::Zero();
  belief_anchor_point = anchor_point;
  belief_velocity = selected_measured_velocity;
  belief_acceleration = selected_measured_acceleration;
  belief_velocity_gain = Eigen::Vector3d::Zero();
  velocity_covariance = Eigen::Matrix3d::Zero();
  belief_velocity_online_covariance = Eigen::Matrix3d::Zero();

  state = Eigen::Vector4d::Zero();
  measurement_covariance = Eigen::Matrix4d::Zero();
  state_covariance = Eigen::Matrix4d::Zero();

  motion_score = Eigen::Vector3d(1, 1, 1);

  direction_state = Eigen::Vector3d::Zero();
  direction_state_covariance = Eigen::Matrix3d::Zero();
  output_angular = Eigen::Vector3d::Zero();
  direction_convergence_confidence = 0.0;
  direction_converged = false;

  // output reset
  output_velocity = belief_velocity;
  output_velocity_uncertainty = belief_velocity_online_covariance;
  output_direction = direction;
  output_center = center;
  output_size = size;

  // sensor info reset
  sensor_info.Reset();
  timestamp = 0.0;
}

void TrackedObject::Reset(base::ObjectPtr obj_ptr, const Eigen::Affine3d& pose,
                          const Eigen::Vector3d& global_to_local_offset,
                          const base::SensorInfo& sensor, double timestamp) {
  Reset();
  AttachObject(obj_ptr, pose, global_to_local_offset, sensor, timestamp);
}

void TrackedObject::CopyFrom(TrackedObjectPtr rhs, bool is_deep) {
  *this = *rhs;
  if (is_deep) {
    object_ptr = base::ObjectPool::Instance().Get();
    *object_ptr = *(rhs->object_ptr);
  } else {
    object_ptr = rhs->object_ptr;
  }
}

float TrackedObject::GetVelThreshold(base::ObjectPtr obj) const {
  if (obj->type == base::ObjectType::VEHICLE) {
    return 0.99f;
  }
  return 0.0f;
}

void TrackedObject::ToObject(base::ObjectPtr obj) const {
  *obj = *object_ptr;
  // obj id keep default
  // obj polygon calculate outside, because
  /* 1. if ConvexHull2D as member variable:
           constructor time consume a little large
     2. if ConvexHull2D as static or global variable:
           need mutex time consume in multi threads
     what a pity!
  */
  obj->direction = output_direction.cast<float>();
  // obj theta_variance not calculate in tracker, keep default
  obj->center = output_center;
  // obj center_uncertainty not calculate in tracker, keep default
  obj->size = output_size.cast<float>();
  // obj size_varuance not calculate in tracker, keep default
  obj->anchor_point = belief_anchor_point;
  obj->type = type;
  // obj type_probs calculate in tracker
  obj->type_probs = type_probs;
  // obj confidence not calculate in tracker, keep default
  obj->track_id = track_id;
  obj->velocity = output_velocity.cast<float>();
  obj->velocity_uncertainty = output_velocity_uncertainty.cast<float>();
  obj->velocity_converged = converged;
  obj->tracking_time = tracking_time;
  if (obj->velocity.norm() > GetVelThreshold(obj)) {
    obj->theta = std::atan2(obj->velocity[1], obj->velocity[0]);
  } else {
    obj->theta = std::atan2(obj->direction[1], obj->direction[0]);
  }
  // obj latest_tracked_time not calculate in tracker, keep default
  // obj car_light not calculate in tracker, keep default
  // obj lidar_supplement cloud_world has passed in *obj = *object_ptr
  // obj lidar_supplement other elements not calculate in tracker, keep default
  // obj radar_supplement not calculate in tracker, keep default
  // obj camera_supplement not calculate in tracker, keep default
}

std::string TrackedObject::ToString() const {
  // std::string txt;
  // return txt;
  std::ostringstream oos;
  oos << "obj id: " << object_ptr->id << ", track_id: " << object_ptr->track_id
      << ", bary_center: (" << barycenter[0] << "," << barycenter[1] << ","
      << barycenter[2] << ")"
      << ", lane_direction: (" << lane_direction[0] << "," << lane_direction[1]
      << "," << lane_direction[2] << ")";
  return oos.str();
}

void TrackedObject::ComputeShapeFeatures() {
  // Compute object's shape feature
  // 1. check whether shape feature is ready
  // 2. compute object's shape feature
  shape_features_full.resize(histogram_bin_size * 7);
  FeatureDescriptor fd(&(object_ptr->lidar_supplement.cloud));
  fd.ComputeHistogram(static_cast<int>(histogram_bin_size),
                      shape_features_full.data());

  size_t feature_len = histogram_bin_size * 3;
  shape_features.clear();
  shape_features.resize(feature_len);
  for (size_t i = 0; i < feature_len; ++i) {
    shape_features[i] = shape_features_full[i + 7];
  }
}
}  // namespace lidar
}  // namespace perception
}  // namespace apollo
