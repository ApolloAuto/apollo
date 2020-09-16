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

#include "modules/v2x/fusion/apps/common/trans_tools.h"

#include <limits>

namespace apollo {
namespace v2x {
namespace ft {

void Pb2Object(const PerceptionObstacle &obstacle, base::Object *object,
               const std::string &frame_id, double timestamp_object) {
  Eigen::Vector3d value;
  Eigen::Matrix3d variance;
  variance.setIdentity();
  object->type_probs.push_back(0.85);
  object->sub_type_probs.push_back(0.85);
  object->type = static_cast<base::ObjectType>(obstacle.type());
  variance = variance * 1;
  switch (obstacle.sub_type()) {
    case PerceptionObstacle::ST_UNKNOWN:
      object->sub_type = base::ObjectSubType::UNKNOWN;
      variance = variance * 3;
      break;

    case PerceptionObstacle::ST_UNKNOWN_MOVABLE:
      object->sub_type = base::ObjectSubType::UNKNOWN_MOVABLE;
      variance = variance * 2;
      break;

    case PerceptionObstacle::ST_CAR:
      object->sub_type = base::ObjectSubType::CAR;
      variance = variance * 0.8;
      break;

    case PerceptionObstacle::ST_VAN:
      object->sub_type = base::ObjectSubType::VAN;
      variance = variance * 1;
      break;

    case PerceptionObstacle::ST_TRUCK:
      object->sub_type = base::ObjectSubType::TRUCK;
      variance = variance * 3;
      break;

    case PerceptionObstacle::ST_BUS:
      object->sub_type = base::ObjectSubType::BUS;
      variance = variance * 3;
      break;

    case PerceptionObstacle::ST_CYCLIST:
      object->sub_type = base::ObjectSubType::CYCLIST;
      variance = variance * 0.8;
      break;

    case PerceptionObstacle::ST_MOTORCYCLIST:
      object->sub_type = base::ObjectSubType::MOTORCYCLIST;
      variance = variance * 0.8;
      break;

    case PerceptionObstacle::ST_TRICYCLIST:
      object->sub_type = base::ObjectSubType::TRICYCLIST;
      variance = variance * 0.8;
      break;

    case PerceptionObstacle::ST_PEDESTRIAN:
      object->sub_type = base::ObjectSubType::PEDESTRIAN;
      variance = variance * 0.8;
      break;

    case PerceptionObstacle::ST_TRAFFICCONE:
      object->sub_type = base::ObjectSubType::TRAFFICCONE;
      variance = variance * 0.8;
      break;

    default:
      break;
  }
  if (obstacle.has_timestamp()) {
    object->timestamp = obstacle.timestamp();
  } else {
    object->timestamp = timestamp_object;
  }
  value << obstacle.position().x(), obstacle.position().y(), 0.0;
  object->position.Set(value, variance);
  value << obstacle.velocity().x(), obstacle.velocity().y(),
      obstacle.velocity().z();
  object->velocity.Set(value, variance);
  object->theta.Set(obstacle.theta(), 0.5);
  object->sensor_type = base::SensorType::MONOCULAR_CAMERA;
  object->track_id = obstacle.id();
  object->frame_id = frame_id;
  variance.setIdentity();
  value << obstacle.length(), obstacle.width(), obstacle.height();
  object->size.Set(value, variance);
  std::vector<base::Info3d> polygon_info3d;
  for (auto &polygon_point : obstacle.polygon_point()) {
    base::Info3d point;
    value << polygon_point.x(), polygon_point.y(), polygon_point.z();
    point.Set(value, variance);
    polygon_info3d.push_back(point);
  }
  object->polygon = polygon_info3d;
}

void V2xPb2Object(const apollo::v2x::V2XObstacle &obstacle,
                  base::Object *object, const std::string &frame_id,
                  double timestamp_object) {
  Pb2Object(obstacle.perception_obstacle(), object, frame_id, timestamp_object);
  if (obstacle.has_v2x_info() && obstacle.v2x_info().v2x_type_size() > 0 &&
      obstacle.v2x_info().v2x_type(0) ==
          ::apollo::v2x::V2XInformation::ZOMBIES_CAR) {
    object->v2x_type = base::V2xType::ZOMBIES_CAR;
  }
}

base::Object Pb2Object(const PerceptionObstacle &obstacle,
                       const std::string &frame_id) {
  base::Object object;
  Eigen::Vector3d value;
  Eigen::Matrix3d variance;
  variance.setIdentity();
  object.timestamp = obstacle.timestamp();
  // object
  value << obstacle.position().x(), obstacle.position().y(),
      obstacle.position().z();

  object.position.Set(value, variance);
  value << obstacle.velocity().x(), obstacle.velocity().y(),
      obstacle.velocity().z();
  object.velocity.Set(value, variance);
  object.theta.Set(obstacle.theta(), 0.5);
  object.sensor_type = base::SensorType::MONOCULAR_CAMERA;
  object.track_id = obstacle.id();
  object.frame_id = frame_id;
  value << obstacle.length(), obstacle.width(), obstacle.height();
  object.size.Set(value, variance);
  object.type_probs.push_back(0.85);
  object.sub_type_probs.push_back(0.85);
  object.type = static_cast<base::ObjectType>(obstacle.type());
  switch (obstacle.sub_type()) {
    case PerceptionObstacle::ST_UNKNOWN:
      object.sub_type = base::ObjectSubType::UNKNOWN;
      break;

    case PerceptionObstacle::ST_UNKNOWN_MOVABLE:
      object.sub_type = base::ObjectSubType::UNKNOWN_MOVABLE;
      break;

    case PerceptionObstacle::ST_CAR:
      object.sub_type = base::ObjectSubType::CAR;
      break;

    case PerceptionObstacle::ST_VAN:
      object.sub_type = base::ObjectSubType::VAN;
      break;

    case PerceptionObstacle::ST_TRUCK:
      object.sub_type = base::ObjectSubType::TRUCK;
      break;

    case PerceptionObstacle::ST_BUS:
      object.sub_type = base::ObjectSubType::BUS;
      break;

    case PerceptionObstacle::ST_CYCLIST:
      object.sub_type = base::ObjectSubType::CYCLIST;
      break;

    case PerceptionObstacle::ST_MOTORCYCLIST:
      object.sub_type = base::ObjectSubType::MOTORCYCLIST;
      break;

    case PerceptionObstacle::ST_TRICYCLIST:
      object.sub_type = base::ObjectSubType::TRICYCLIST;
      break;

    case PerceptionObstacle::ST_PEDESTRIAN:
      object.sub_type = base::ObjectSubType::PEDESTRIAN;
      break;

    case PerceptionObstacle::ST_TRAFFICCONE:
      object.sub_type = base::ObjectSubType::TRAFFICCONE;
      break;

    default:
      break;
  }
  return object;
}

PerceptionObstacle Object2Pb(const base::Object &object) {
  PerceptionObstacle obstacle;
  // times
  obstacle.set_timestamp(object.timestamp);
  // id
  obstacle.set_id(object.track_id);
  // position
  obstacle.mutable_position()->set_x(object.position.x());
  obstacle.mutable_position()->set_y(object.position.y());
  obstacle.mutable_position()->set_z(object.position.z());
  // velocity
  obstacle.mutable_velocity()->set_x(object.velocity.x());
  obstacle.mutable_velocity()->set_y(object.velocity.y());
  obstacle.mutable_velocity()->set_z(object.velocity.z());
  // yaw
  obstacle.set_theta(object.theta.Value());
  // lwh
  obstacle.set_length(object.size.length());
  obstacle.set_width(object.size.width());
  obstacle.set_height(object.size.height());
  obstacle.set_type(static_cast<PerceptionObstacle::Type>(object.type));
  switch (object.sub_type) {
    case base::ObjectSubType::UNKNOWN:
      obstacle.set_sub_type(PerceptionObstacle::ST_UNKNOWN);
      break;

    case base::ObjectSubType::UNKNOWN_MOVABLE:
      obstacle.set_sub_type(PerceptionObstacle::ST_UNKNOWN_MOVABLE);
      break;

    case base::ObjectSubType::CAR:
      obstacle.set_sub_type(PerceptionObstacle::ST_CAR);
      break;

    case base::ObjectSubType::VAN:
      obstacle.set_sub_type(PerceptionObstacle::ST_VAN);
      break;

    case base::ObjectSubType::TRUCK:
      obstacle.set_sub_type(PerceptionObstacle::ST_TRUCK);
      break;

    case base::ObjectSubType::BUS:
      obstacle.set_sub_type(PerceptionObstacle::ST_BUS);
      break;

    case base::ObjectSubType::CYCLIST:
      obstacle.set_sub_type(PerceptionObstacle::ST_CYCLIST);
      break;

    case base::ObjectSubType::MOTORCYCLIST:
      obstacle.set_sub_type(PerceptionObstacle::ST_MOTORCYCLIST);
      break;

    case base::ObjectSubType::TRICYCLIST:
      obstacle.set_sub_type(PerceptionObstacle::ST_TRICYCLIST);
      break;

    case base::ObjectSubType::PEDESTRIAN:
      obstacle.set_sub_type(PerceptionObstacle::ST_PEDESTRIAN);
      break;

    case base::ObjectSubType::TRAFFICCONE:
      obstacle.set_sub_type(PerceptionObstacle::ST_TRAFFICCONE);
      break;

    default:
      break;
  }
  return obstacle;
}

void FillObjectPolygonFromBBox3D(PerceptionObstacle *object_ptr) {
  struct PolygoPoint {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
  };

  if (!object_ptr) {
    return;
  }
  const double length = object_ptr->length();
  const double width = object_ptr->width();
  double hl = length / 2;
  double hw = width / 2;
  double cos_theta = std::cos(object_ptr->theta());
  double sin_theta = std::sin(object_ptr->theta());
  PolygoPoint polygon[4];
  polygon[0].x = hl * cos_theta - hw * sin_theta + object_ptr->position().x();
  polygon[0].y = hl * sin_theta + hw * cos_theta + object_ptr->position().y();
  polygon[0].z = object_ptr->position().z();
  polygon[1].x = hl * cos_theta + hw * sin_theta + object_ptr->position().x();
  polygon[1].y = hl * sin_theta - hw * cos_theta + object_ptr->position().y();
  polygon[1].z = object_ptr->position().z();
  polygon[2].x = -hl * cos_theta + hw * sin_theta + object_ptr->position().x();
  polygon[2].y = -hl * sin_theta - hw * cos_theta + object_ptr->position().y();
  polygon[2].z = object_ptr->position().z();
  polygon[3].x = -hl * cos_theta - hw * sin_theta + object_ptr->position().x();
  polygon[3].y = -hl * sin_theta + hw * cos_theta + object_ptr->position().y();
  polygon[3].z = object_ptr->position().z();
  for (PolygoPoint point : polygon) {
    auto polygon_point = object_ptr->add_polygon_point();
    polygon_point->set_x(point.x);
    polygon_point->set_y(point.y);
    polygon_point->set_z(point.z);
  }
}

void Object2Pb(const base::Object &object, PerceptionObstacle *obstacle) {
  // times
  obstacle->set_timestamp(object.timestamp);
  // id
  obstacle->set_id(object.track_id);
  // position
  obstacle->mutable_position()->set_x(object.position.x());
  obstacle->mutable_position()->set_y(object.position.y());
  obstacle->mutable_position()->set_z(object.position.z());
  // velocity
  obstacle->mutable_velocity()->set_x(object.velocity.x());
  obstacle->mutable_velocity()->set_y(object.velocity.y());
  obstacle->mutable_velocity()->set_z(object.velocity.z());
  // yaw
  obstacle->set_theta(object.theta.Value());
  // lwh
  obstacle->set_length(object.size.length());
  obstacle->set_width(object.size.width());
  obstacle->set_height(object.size.height());
  FillObjectPolygonFromBBox3D(obstacle);
  obstacle->set_type(static_cast<PerceptionObstacle::Type>(object.type));
  switch (object.sub_type) {
    case base::ObjectSubType::UNKNOWN:
      obstacle->set_sub_type(PerceptionObstacle::ST_UNKNOWN);
      break;

    case base::ObjectSubType::UNKNOWN_MOVABLE:
      obstacle->set_sub_type(PerceptionObstacle::ST_UNKNOWN_MOVABLE);
      break;

    case base::ObjectSubType::UNKNOWN_UNMOVABLE:
      obstacle->set_sub_type(PerceptionObstacle::ST_UNKNOWN_UNMOVABLE);
      break;

    case base::ObjectSubType::CAR:
      obstacle->set_sub_type(PerceptionObstacle::ST_CAR);
      break;

    case base::ObjectSubType::VAN:
      obstacle->set_sub_type(PerceptionObstacle::ST_VAN);
      break;

    case base::ObjectSubType::TRUCK:
      obstacle->set_sub_type(PerceptionObstacle::ST_TRUCK);
      break;

    case base::ObjectSubType::BUS:
      obstacle->set_sub_type(PerceptionObstacle::ST_BUS);
      break;

    case base::ObjectSubType::CYCLIST:
      obstacle->set_sub_type(PerceptionObstacle::ST_CYCLIST);
      break;

    case base::ObjectSubType::MOTORCYCLIST:
      obstacle->set_sub_type(PerceptionObstacle::ST_MOTORCYCLIST);
      break;

    case base::ObjectSubType::TRICYCLIST:
      obstacle->set_sub_type(PerceptionObstacle::ST_TRICYCLIST);
      break;

    case base::ObjectSubType::PEDESTRIAN:
      obstacle->set_sub_type(PerceptionObstacle::ST_PEDESTRIAN);
      break;

    case base::ObjectSubType::TRAFFICCONE:
      obstacle->set_sub_type(PerceptionObstacle::ST_TRAFFICCONE);
      break;

    default:
      break;
  }
  obstacle->set_source(PerceptionObstacle::HOST_VEHICLE);
  if (object.v2x_type == base::V2xType::ZOMBIES_CAR) {
    obstacle->mutable_v2x_info()->add_v2x_type(
        ::apollo::perception::V2XInformation::ZOMBIES_CAR);
    obstacle->set_source(PerceptionObstacle::V2X);
  }
  if (object.v2x_type == base::V2xType::BLIND_ZONE) {
    obstacle->mutable_v2x_info()->add_v2x_type(
        ::apollo::perception::V2XInformation::BLIND_ZONE);
    obstacle->set_source(PerceptionObstacle::V2X);
  }
}

void Object2V2xPb(const base::Object &object, V2XObstacle *obstacle) {
  PerceptionObstacle perception_obstacle;
  Object2Pb(object, &perception_obstacle);
  obstacle->mutable_perception_obstacle()->CopyFrom(perception_obstacle);
}

double Pbs2Objects(const PerceptionObstacles &obstacles,
                   std::vector<base::Object> *objects,
                   const std::string &frame_id) {
  double timestamp = std::numeric_limits<double>::max();
  objects->clear();
  double timestamp_object = 0.0;
  if (obstacles.perception_obstacle_size() > 0 &&
      obstacles.perception_obstacle(0).has_timestamp() == false) {
    if (obstacles.header().has_camera_timestamp() &&
        obstacles.header().camera_timestamp() > 10.0) {
      timestamp_object = obstacles.header().camera_timestamp() / 1.0e9;
    } else {
      timestamp_object = obstacles.header().lidar_timestamp() / 1.0e9;
    }
  }
  for (int j = 0; j < obstacles.perception_obstacle_size(); ++j) {
    base::Object object;
    Pb2Object(obstacles.perception_obstacle(j), &object, frame_id,
              timestamp_object);
    objects->push_back(object);
    if (timestamp > object.timestamp) {
      timestamp = object.timestamp;
    }
  }

  return timestamp;
}

void CarstatusPb2Object(const LocalizationEstimate &carstatus,
                        base::Object *object, const std::string &frame_id) {
  Eigen::Vector3d value;
  Eigen::Matrix3d variance;
  variance.setIdentity();
  object->type_probs.push_back(0.85);
  object->sub_type_probs.push_back(0.85);
  object->type = base::ObjectType::VEHICLE;
  object->sub_type = base::ObjectSubType::CAR;
  object->v2x_type = base::V2xType::HOST_VEHICLE;
  variance = variance * 0.8;
  value << carstatus.pose().position().x(), carstatus.pose().position().y(),
      0.0;
  object->position.Set(value, variance);
  value << carstatus.pose().linear_velocity().x(),
      carstatus.pose().linear_velocity().y(),
      carstatus.pose().linear_velocity().z();
  object->velocity.Set(value, variance);
  object->theta.Set(carstatus.pose().heading(), 0.5);
  object->sensor_type = base::SensorType::MONOCULAR_CAMERA;
  object->track_id = 0;
  object->frame_id = frame_id;
  variance.setIdentity();
  value << 5.02203, 2.13135, 2.17711;
  object->size.Set(value, variance);
  object->timestamp = carstatus.header().timestamp_sec();
}

double V2xPbs2Objects(const V2XObstacles &obstacles,
                      std::vector<base::Object> *objects,
                      const std::string &frame_id) {
  double timestamp = std::numeric_limits<double>::max();
  objects->clear();
  double timestamp_object = 0.0;
  if (obstacles.v2x_obstacle_size() > 0 &&
      obstacles.v2x_obstacle(0).perception_obstacle().has_timestamp() ==
          false) {
    if (obstacles.header().has_camera_timestamp()) {
      timestamp_object = obstacles.header().camera_timestamp() / 1000000000.0;
    } else {
      timestamp_object = obstacles.header().lidar_timestamp() / 1000000000.0;
    }
  }
  for (int j = 0; j < obstacles.v2x_obstacle_size(); ++j) {
    base::Object object;
    V2xPb2Object(obstacles.v2x_obstacle(j), &object, frame_id,
                 timestamp_object);
    objects->push_back(object);
    if (timestamp > object.timestamp) {
      timestamp = object.timestamp;
    }
  }

  return timestamp;
}

void Objects2Pbs(const std::vector<base::Object> &objects,
                 std::shared_ptr<PerceptionObstacles> obstacles) {
  obstacles->mutable_perception_obstacle()->Clear();
  if (objects.size() < 1) {
    return;
  }
  // obstacles->mutable_header()->set_frame_id(objects[0].frame_id);
  for (const auto &object : objects) {
    if (object.v2x_type == base::V2xType::HOST_VEHICLE) {
      continue;
    }
    PerceptionObstacle obstacle;
    Object2Pb(object, &obstacle);
    obstacles->add_perception_obstacle()->CopyFrom(obstacle);
  }
}

void Objects2V2xPbs(const std::vector<base::Object> &objects,
                    std::shared_ptr<V2XObstacles> obstacles) {
  obstacles->mutable_v2x_obstacle()->Clear();
  if (objects.size() < 1) {
    return;
  }
  for (const auto &object : objects) {
    if (object.v2x_type == base::V2xType::HOST_VEHICLE) {
      continue;
    }
    V2XObstacle obstacle;
    Object2V2xPb(object, &obstacle);
    obstacles->add_v2x_obstacle()->CopyFrom(obstacle);
  }
}

}  // namespace ft
}  // namespace v2x
}  // namespace apollo
