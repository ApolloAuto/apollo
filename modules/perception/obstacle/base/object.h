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

#ifndef MODULES_PERCEPTION_OBSTACLE_BASE_OBJECT_H_
#define MODULES_PERCEPTION_OBSTACLE_BASE_OBJECT_H_

#include <memory>
#include <string>
#include <vector>

#include "Eigen/Core"

#include "modules/common/proto/error_code.pb.h"
#include "modules/perception/lib/base/time_util.h"
#include "modules/perception/lib/pcl_util/pcl_types.h"
#include "modules/perception/obstacle/base/object_supplement.h"
#include "modules/perception/obstacle/base/types.h"
#include "modules/perception/proto/perception_obstacle.pb.h"

namespace apollo {
namespace perception {

struct alignas(16) Object {
  Object();
  // deep copy
  void clone(const Object& rhs);
  std::string ToString() const;

  void Serialize(PerceptionObstacle* pb_obj) const;
  void Deserialize(const PerceptionObstacle& pb_obs);

  // object id per frame
  int id = 0;
  // point cloud of the object
  pcl_util::PointCloudPtr cloud;
  // convex hull of the object
  PolygonDType polygon;

  // oriented boundingbox information
  // main direction
  Eigen::Vector3d direction;
  // the yaw angle, theta = 0.0 <=> direction = (1, 0, 0)
  double theta = 0.0;
  // ground center of the object (cx, cy, z_min)
  Eigen::Vector3d center;
  // size of the oriented bbox, length is the size in the main direction
  double length = 0.0;
  double width = 0.0;
  double height = 0.0;
  // shape feature used for tracking
  std::vector<float> shape_features;

  // foreground score/probability
  float score = 0.0;
  // foreground score/probability type
  ScoreType score_type = SCORE_CNN;

  // Object classification type.
  ObjectType type;
  // Probability of each type, used for track type.
  std::vector<float> type_probs;

  // fg/bg flag
  bool is_background = false;

  // tracking information
  int track_id = 0;
  Eigen::Vector3d velocity;
  // age of the tracked object
  double tracking_time = 0.0;
  double latest_tracked_time = 0.0;

  // stable anchor_point during time, e.g., barycenter
  Eigen::Vector3d anchor_point;

  // noise covariance matrix for uncertainty of position and velocity
  Eigen::Matrix3d position_uncertainty;
  Eigen::Matrix3d velocity_uncertainty;

  // sensor particular suplplements, default nullptr
  RadarSupplementPtr radar_supplement = nullptr;
};

typedef std::shared_ptr<Object> ObjectPtr;
typedef std::shared_ptr<const Object> ObjectConstPtr;

// Sensor single frame objects.
struct SensorObjects {
  SensorObjects() {
    sensor2world_pose = Eigen::Matrix4d::Zero();
  }

  std::string ToString() const;

  // Transmit error_code to next subnode.
  common::ErrorCode error_code = common::ErrorCode::OK;

  SensorType sensor_type = UNKNOWN_SENSOR_TYPE;
  std::string sensor_id;
  double timestamp = 0.0;
  SeqId seq_num = 0;
  std::vector<ObjectPtr> objects;
  Eigen::Matrix4d sensor2world_pose;
};

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_BASE_OBJECT_H_
