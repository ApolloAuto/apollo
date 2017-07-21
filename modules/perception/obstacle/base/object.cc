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

#include <sstream>

#include "modules/common/macro.h"
#include "modules/common/log.h"

namespace apollo {
namespace perception {
namespace obstacle {

// using apollo::common::perception::PerceptionObstacle;
// using apollo::common::perception::PerceptionErrorCode_Name;
// using apollo::common::perception::Point;
using std::ostringstream;
using std::string;
using std::vector;
using Eigen::Vector3d;

Object::Object() {
  direction = Vector3d(1, 0, 0);
  center = Vector3d::Zero();
  velocity = Vector3d::Zero();
  cloud.reset(new PointCloud);
  type_probs.resize(MAX_OBJECT_TYPE, 0);
}

Object::Object(const Object& rhs) {
  id = rhs.id;
  cloud = rhs.cloud;
  polygon = rhs.polygon;
  direction = rhs.direction;
  theta = rhs.theta;
  center = rhs.center;
  length = rhs.length;
  width = rhs.width;
  height = rhs.height;
  type = rhs.type;
  type_probs = rhs.type_probs;
  is_background = rhs.is_background;
  track_id = rhs.track_id;
  velocity = rhs.velocity;
  tracking_time = rhs.tracking_time;
  latest_tracked_time = rhs.latest_tracked_time;
}

Object& Object::operator = (const Object& rhs) {
  id = rhs.id;
  cloud = rhs.cloud;
  polygon = rhs.polygon;
  direction = rhs.direction;
  theta = rhs.theta;
  center = rhs.center;
  length = rhs.length;
  width = rhs.width;
  height = rhs.height;
  type = rhs.type;
  type_probs = rhs.type_probs;
  is_background = rhs.is_background;
  track_id = rhs.track_id;
  velocity = rhs.velocity;
  tracking_time = rhs.tracking_time;
  latest_tracked_time = rhs.latest_tracked_time;
  return (*this);
}

void Object::clone(const Object& rhs) {
  id = rhs.id;
  pcl::copyPointCloud<Point, Point>(*(rhs.cloud), *cloud);
  polygon = rhs.polygon;
  direction = rhs.direction;
  theta = rhs.theta;
  center = rhs.center;
  length = rhs.length;
  width = rhs.width;
  height = rhs.height;
  type = rhs.type;
  type_probs = rhs.type_probs;
  is_background = rhs.is_background;
  track_id = rhs.track_id;
  velocity = rhs.velocity;
  tracking_time = rhs.tracking_time;
  latest_tracked_time = rhs.latest_tracked_time;
}

string Object::to_string() const {
  ostringstream oss;
  oss << "Object[id: " << id << ", track_id: " << track_id
    << ", cloud_size: " << cloud->size() << ", direction: "
    << direction.transpose() << ", center: " << center.transpose()
    << " velocity: " << velocity.transpose() << ", width: " << width
    << ", length: " << length << ", height: " << height
    << ", polygon_size: " << polygon.size()
    << ", type: " << type << ", is_background: " << is_background << "]";
    // << ", tracking_time: " << GLOG_TIMESTAMP(tracking_time)
    // << ", latest_tracked_time: " << GLOG_TIMESTAMP(latest_tracked_time) << "]";

  return oss.str();
}

// bool Object::serialize(PerceptionObstacle* pb_obj) const {
//   CHECK(pb_obj != NULL);
//   pb_obj->set_id(track_id);
//   pb_obj->set_theta(theta);
//
//   Point* obj_center = pb_obj->mutable_position();
//   obj_center->set_x(center(0));
//   obj_center->set_y(center(1));
//   obj_center->set_z(center(2));
//
//   Point* obj_velocity = pb_obj->mutable_velocity();
//   obj_velocity->set_x(velocity(0));
//   obj_velocity->set_y(velocity(1));
//   obj_velocity->set_z(velocity(2));
//
//   pb_obj->set_length(length);
//   pb_obj->set_width(width);
//   pb_obj->set_height(height);
//
//   for (auto point : polygon.points) {
//     Point* p = pb_obj->add_polygon_point();
//     p->set_x(point.x);
//     p->set_y(point.y);
//     p->set_z(point.z);
//   }
//
//   for (auto point : cloud->points) {
//     pb_obj->add_point_cloud(point.x);
//     pb_obj->add_point_cloud(point.y);
//     pb_obj->add_point_cloud(point.z);
//   }
//
//   Point* obj_anchor_point = pb_obj->mutable_anchor_point();
//   obj_anchor_point->set_x(anchor_point(0));
//   obj_anchor_point->set_y(anchor_point(1));
//   obj_anchor_point->set_z(anchor_point(2));
//
//   for (int i = 0; i < 3; i++) {
//     for (int j = 0; j < 3; j++) {
//       pb_obj->add_position_covariance(position_uncertainty(i, j));
//       pb_obj->add_velocity_covariance(velocity_uncertainty(i, j));
//     }
//   }
//
//   pb_obj->set_tracking_time(tracking_time);
//   pb_obj->set_type(static_cast<PerceptionObstacle::Type>(type));
//   pb_obj->set_timestamp(latest_tracked_time);  // in seconds.
//
//   return true;
// }
//
// bool Object::deserialize(const PerceptionObstacle& pb_obs) {
//   track_id = pb_obs.id();
//   theta = pb_obs.theta();
//
//   center(0) = pb_obs.position().x();
//   center(1) = pb_obs.position().y();
//   center(2) = pb_obs.position().z();
//
//   velocity(0) = pb_obs.velocity().x();
//   velocity(1) = pb_obs.velocity().y();
//   velocity(2) = pb_obs.velocity().z();
//
//   length = pb_obs.length();
//   width = pb_obs.width();
//   height = pb_obs.height();
//
//   polygon.clear();
//   for (int idx = 0; idx < pb_obs.polygon_point_size(); ++idx) {
//     const auto& p = pb_obs.polygon_point(idx);
//     PointD point;
//     point.x = p.x();
//     point.y = p.y();
//     point.z = p.z();
//     polygon.push_back(point);
//   }
//
//   tracking_time = pb_obs.tracking_time();
//   latest_tracked_time = pb_obs.timestamp();
//   type = static_cast<ObjectType>(pb_obs.type());
//
//   return true;
// }

}  // namespace obstacle
}  // namespace perception
}  // namespace apollo
