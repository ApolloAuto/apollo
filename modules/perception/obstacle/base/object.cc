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

#include "modules/perception/obstacle/base/object.h"

#include "modules/common/log.h"
#include "modules/common/macro.h"
#include "modules/common/math/box2d.h"
#include "modules/common/util/string_util.h"
#include "modules/perception/common/perception_gflags.h"

namespace apollo {
namespace perception {

using apollo::common::math::Box2d;
using apollo::common::math::Vec2d;
using apollo::common::util::Print;
using apollo::common::util::StrCat;
using Eigen::Vector3d;

Object::Object() {
  cloud.reset(new pcl_util::PointCloud);
  type_probs.resize(static_cast<int>(ObjectType::MAX_OBJECT_TYPE), 0);
  position_uncertainty << 0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01;
  velocity_uncertainty << 0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01;
}

void Object::clone(const Object& rhs) {
  *this = rhs;
  pcl::copyPointCloud<pcl_util::Point, pcl_util::Point>(*(rhs.cloud), *cloud);
  radar_supplement = nullptr;
  if (rhs.radar_supplement != nullptr) {
    radar_supplement.reset(new RadarSupplement(*rhs.radar_supplement));
  }
  camera_supplement = nullptr;
  if (rhs.camera_supplement != nullptr) {
    camera_supplement.reset(new CameraSupplement());
    camera_supplement->clone(*(rhs.camera_supplement));
  }
}

std::string Object::ToString() const {
  // StrCat supports 9 arguments at most.
  return StrCat(StrCat("Object[id: ", id,
                       ", "
                       "track_id: ",
                       track_id,
                       ", "
                       "cloud_size: ",
                       cloud->size(),
                       ", "
                       "direction: ",
                       Print(direction.transpose()), ", "),
                StrCat("center: ", Print(center.transpose()),
                       ", "
                       "velocity: ",
                       Print(velocity.transpose()),
                       ", "
                       "width: ",
                       width,
                       ", "
                       "length: ",
                       length, ", "),
                StrCat("height: ", height,
                       ", "
                       "polygon_size: ",
                       polygon.size(),
                       ", "
                       "type: ",
                       static_cast<int>(type),
                       ", "
                       "is_background: ",
                       is_background),
                StrCat(", is_cipv: ", b_cipv, "]"));
}

// Add 4 corners in the polygon
void Object::AddFourCorners(PerceptionObstacle* pb_obj) const {
  Box2d object_bounding_box = {{center(0), center(1)}, theta, length, width};
  std::vector<Vec2d> corners;
  object_bounding_box.GetAllCorners(&corners);

  for (const auto& corner : corners) {
    Point* p = pb_obj->add_polygon_point();
    p->set_x(corner.x());
    p->set_y(corner.y());
    p->set_z(0.0);
  }
  ADEBUG << "PerceptionObstacle bounding box is : "
         << object_bounding_box.DebugString();
}

void Object::Serialize(PerceptionObstacle* pb_obj) const {
  CHECK(pb_obj != nullptr);
  pb_obj->set_id(track_id);
  pb_obj->set_theta(theta);

  Point* obj_center = pb_obj->mutable_position();
  obj_center->set_x(center(0));
  obj_center->set_y(center(1));
  obj_center->set_z(center(2));

  Point* obj_velocity = pb_obj->mutable_velocity();
  obj_velocity->set_x(velocity(0));
  obj_velocity->set_y(velocity(1));
  obj_velocity->set_z(velocity(2));

  pb_obj->set_length(length);
  pb_obj->set_width(width);
  pb_obj->set_height(height);

  if (polygon.size() /*pb_obs.polygon_point_size() */ >= 4) {
    for (auto point : polygon.points) {
      Point* p = pb_obj->add_polygon_point();
      p->set_x(point.x);
      p->set_y(point.y);
      p->set_z(point.z);
    }
  } else {  // if polygon size is less than 4
    // Generate polygon from center position, width, height
    // and orientation of the object
    AddFourCorners(pb_obj);
  }

  if (FLAGS_is_serialize_point_cloud) {
    for (auto point : cloud->points) {
      pb_obj->add_point_cloud(point.x);
      pb_obj->add_point_cloud(point.y);
      pb_obj->add_point_cloud(point.z);
    }
  }

  pb_obj->set_confidence(score);
  pb_obj->set_confidence_type(
      static_cast<PerceptionObstacle::ConfidenceType>(score_type));
  pb_obj->set_tracking_time(tracking_time);
  pb_obj->set_type(static_cast<PerceptionObstacle::Type>(type));
  pb_obj->set_timestamp(latest_tracked_time);  // in seconds.
}

void Object::Deserialize(const PerceptionObstacle& pb_obs) {
  track_id = pb_obs.id();
  theta = pb_obs.theta();

  center(0) = pb_obs.position().x();
  center(1) = pb_obs.position().y();
  center(2) = pb_obs.position().z();

  velocity(0) = pb_obs.velocity().x();
  velocity(1) = pb_obs.velocity().y();
  velocity(2) = pb_obs.velocity().z();

  length = pb_obs.length();
  width = pb_obs.width();
  height = pb_obs.height();

  polygon.clear();
  for (int idx = 0; idx < pb_obs.polygon_point_size(); ++idx) {
    const auto& p = pb_obs.polygon_point(idx);
    pcl_util::PointD point;
    point.x = p.x();
    point.y = p.y();
    point.z = p.z();
    polygon.push_back(point);
  }

  score = pb_obs.confidence();
  score_type = static_cast<ScoreType>(pb_obs.confidence_type());
  tracking_time = pb_obs.tracking_time();
  latest_tracked_time = pb_obs.timestamp();
  type = static_cast<ObjectType>(pb_obs.type());
}

std::string SensorObjects::ToString() const {
  std::ostringstream oss;
  oss << "sensor_type: " << GetSensorType(sensor_type)
      << ", timestamp:" << GLOG_TIMESTAMP(timestamp)
      << ", sensor2world_pose:\n";
  oss << sensor2world_pose << "\n, objects: " << objects.size() << " < ";
  for (auto obj : objects) {
    oss << "\n" << obj->ToString();
  }
  oss << " >]";
  return oss.str();
}

}  // namespace perception
}  // namespace apollo
