/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

#include <Eigen/Core>

#include "absl/strings/str_cat.h"

#include "modules/perception/base/object_supplement.h"
#include "modules/perception/base/object_types.h"
#include "modules/perception/base/sensor_meta.h"

namespace apollo {
namespace v2x {
namespace ft {
namespace base {

using apollo::perception::base::CameraObjectSupplement;
using apollo::perception::base::FusionObjectSupplement;
using apollo::perception::base::ObjectSubType;
using apollo::perception::base::ObjectType;
using apollo::perception::base::SensorType;

enum class V2xType {
  UNKNOWN = 0,
  ZOMBIES_CAR = 1,
  BLIND_ZONE = 2,
  HOST_VEHICLE = 13,
};

enum class MessageType {
  UNKNOWN_MESSAGE_TYPE = -1,
  ROADSIDE = 0,
  VEHICLE = 1,
};

// Value and Variance
template <typename Val, typename Var>
class Info {
 public:
  Info() = default;
  Info(Val val, Var var) {
    value_ = val;
    variance_ = var;
  }
  ~Info() = default;
  // Info(const Info &) = delete;
  Info(const Info &rhs) {
    value_ = rhs.value_;
    variance_ = rhs.variance_;
  }
  // Info &operator=(const Info &) = delete;
  Info &operator=(const Info &rhs) {
    value_ = rhs.value_;
    variance_ = rhs.variance_;
    return *this;
  }

  void Set(Val value, Var variance) {
    value_ = value;
    variance_ = variance;
  }
  Val Value() const { return value_; }

  Var Variance() const { return variance_; }

 protected:
  Val value_;
  Var variance_;
};

class Info3f : public Info<Eigen::Vector3f, Eigen::Matrix3f> {
 public:
  Info3f() {
    value_.setZero();
    variance_.setZero();
  }
  ~Info3f() = default;

  float x() const { return value_(0); }

  float y() const { return value_(1); }

  float z() const { return value_(2); }

  float length() const { return value_(0); }

  float width() const { return value_(1); }

  float height() const { return value_(2); }
};

class Info3d : public Info<Eigen::Vector3d, Eigen::Matrix3d> {
 public:
  double x() const { return value_(0); }

  double y() const { return value_(1); }

  double z() const { return value_(2); }

  double length() const { return value_(0); }

  double width() const { return value_(1); }

  double height() const { return value_(2); }
};

struct alignas(16) Object {
  typedef Info<bool, float> Infob;
  typedef Info<float, float> Infof;
  typedef Info<double, double> Infod;
  typedef Info<Eigen::Vector2f, Eigen::Matrix2f> Info2f;
  typedef Info<Eigen::Vector2d, Eigen::Matrix2d> Info2d;
  typedef Eigen::Vector2f Point2f;
  typedef Eigen::Vector3f Point3f;
  Object() : is_temporary_lost(false, 0.0) {}
  ~Object() = default;
  Object(const Object &) = default;
  Object &operator=(const Object &) = default;
  bool operator<(const Object &rhs) const { return timestamp < rhs.timestamp; }
  bool operator>(const Object &rhs) const { return timestamp > rhs.timestamp; }
  bool operator==(const Object &rhs) const {
    return timestamp == rhs.timestamp;
  }
  std::string ToString() const;
  void Reset();
  // camera, lidar, radar and others
  SensorType sensor_type = SensorType::UNKNOWN_SENSOR_TYPE;
  // ROADSIDE
  MessageType message_type = MessageType::UNKNOWN_MESSAGE_TYPE;
  // @brief sensor-specific object supplements, optional
  CameraObjectSupplement camera_supplement;
  FusionObjectSupplement fusion_supplement;
  // message timestamp
  double timestamp = 0.0;
  // original sensor timestamp
  double sensor_timestamp = 0.0;
  // @brief age of the tracked object, required
  double tracking_time = 0.0;
  // @brief timestamp of latest measurement, required
  double latest_tracked_time = 0.0;
  std::string frame_id = "";
  // @brief track id, required
  int track_id = -1;
  // @breif object id per frame, required
  int global_id = -1;
  // @brief center position of the boundingbox (x, y, z), required
  Info3d position;
  // @brief anchor point, required
  Info3d anchor_point;
  // 0-2Pi from east
  Infod theta;
  // @brief theta variance, required
  Infod theta_variance;
  // @brief main direction of the object, required
  Info3d direction;
  /* @brief size = [length, width, height] of boundingbox
     length is the size of the main direction, required
  */
  Info3d size;
  // @brief convex hull of the object, required
  // Point3f polygon;
  // @brief object type, required
  ObjectType type = ObjectType::UNKNOWN;

  V2xType v2x_type = V2xType::UNKNOWN;
  // @brief type variance, required
  double type_variance = 1.0;
  // @brief probability for each type, required
  std::vector<float> type_probs;
  // @brief object sub-type, optional
  ObjectSubType sub_type = ObjectSubType::UNKNOWN;
  // @brief probability for each sub-type, optional
  std::vector<float> sub_type_probs;

  std::vector<std::vector<Info3d>> tentacles;

  std::vector<Info3d> polygon;

  // tracking information
  // @brief the variance of tracked
  Infod track_variance;
  // @brief velocity of the object, required
  Info3d velocity;
  // @brief acceleration of the object, required
  Info3d acceleration;
  // @brief motion state of the tracked object, required
  Infob is_stationary;

  Infob is_temporary_lost;
  std::string DebugString() const {
    return absl::StrCat("id: ", track_id, ", ",              //
                        "time: ", timestamp, ", ",           //
                        "sensor type: ", sensor_type, ", ",  //
                        "x: ", position.x(), ", ",           //
                        "y: ", position.y(), ", ",           //
                        "vx: ", velocity.x(), ", ",          //
                        "vy: ", velocity.y(), ", ",          //
                        "yaw: ", theta.Value());
  }
};

typedef std::shared_ptr<Object> ObjectPtr;
typedef std::shared_ptr<const Object> ObjectConstPtr;

struct alignas(16) ObjectList {
  ObjectList() = default;
  ~ObjectList() = default;
  ObjectList(const ObjectList &) = default;
  ObjectList &operator=(const ObjectList &) = default;
  bool operator<(const ObjectList &rhs) const {
    return timestamp < rhs.timestamp;
  }
  bool operator>(const ObjectList &rhs) const {
    return timestamp > rhs.timestamp;
  }
  bool operator==(const ObjectList &rhs) const {
    return timestamp == rhs.timestamp;
  }

  SensorType sensor_type;
  // @brief sensor-specific object supplements, optional
  CameraObjectSupplement camera_supplement;
  // LidarObjectSupplement lidar_supplement;
  // RadarObjectSupplement radar_supplement;
  FusionObjectSupplement fusion_supplement;

  // message timestamp
  double timestamp = 0.0;
  // original sensor timestamp
  double sensor_timestamp = 0.0;
  // @brief age of the tracked object, require
  std::string frame_id = "";

  std::vector<ObjectPtr> objects;
};

typedef std::shared_ptr<ObjectList> ObjectListPtr;
typedef std::shared_ptr<const ObjectList> ObjectListConstPtr;

}  // namespace base
}  // namespace ft
}  // namespace v2x
}  // namespace apollo
