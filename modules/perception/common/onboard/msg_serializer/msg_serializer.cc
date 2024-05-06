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
#include "modules/perception/common/onboard/msg_serializer/msg_serializer.h"

#include <limits>

#include "cyber/common/log.h"

#include "cyber/time/clock.h"
#include "modules/perception/common/onboard/common_flags/common_flags.h"
#include "modules/common_msgs/prediction_msgs/feature.pb.h"

using Clock = apollo::cyber::Clock;

namespace {
constexpr float kFloatMax = std::numeric_limits<float>::max();
}  // namespace

namespace apollo {
namespace perception {
namespace onboard {

bool MsgSerializer::SerializeMsg(double timestamp, uint64_t lidar_timestamp,
                                 int seq_num,
                                 const std::vector<base::ObjectPtr> &objects,
                                 const apollo::common::ErrorCode &error_code,
                                 PerceptionObstacles *obstacles) {
  double publish_time = Clock::NowInSeconds();
  ::apollo::common::Header *header = obstacles->mutable_header();
  header->set_timestamp_sec(publish_time);
  header->set_module_name("perception_obstacle");
  header->set_sequence_num(seq_num);
  header->set_lidar_timestamp(lidar_timestamp);
  header->set_camera_timestamp(0);
  header->set_radar_timestamp(0);

  obstacles->set_error_code(error_code);
  for (const auto &obj : objects) {
    PerceptionObstacle *obstacle = obstacles->add_perception_obstacle();
    if (!ConvertObjectToPb(obj, obstacle)) {
      AERROR << "ConvertObjectToPb failed, Object:" << obj->ToString();
      return false;
    }
  }
  return true;
}

bool MsgSerializer::SerializeBenchmarkMsg(
    double timestamp, uint64_t lidar_timestamp,
    int seq_num,
    const std::vector<base::ObjectPtr>& objects,
    PerceptionBenchmarkFrame* benchmark_frame) {
  // double publish_time = Clock::NowInSeconds();
  ::apollo::common::Header *header = benchmark_frame->mutable_header();
  header->set_timestamp_sec(timestamp);
  header->set_module_name("perception_benchmark");
  header->set_sequence_num(seq_num);
  header->set_lidar_timestamp(lidar_timestamp);
  header->set_camera_timestamp(0);
  header->set_radar_timestamp(0);

  for (const auto &obj : objects) {
    PerceptionObstacle *obstacle = benchmark_frame->add_perception_obstacle();
    if (!ConvertObjectToPb(obj, obstacle)) {
      AERROR << "ConvertObjectToPb failed, Object:" << obj->ToString();
      return false;
    }
  }
  return true;
}

bool MsgSerializer::SerializeLidarFrameMsg(
    double timestamp, uint64_t lidar_timestamp, int seq_num,
    const Eigen::Affine3d &pose, const std::vector<base::ObjectPtr> &objects,
    PerceptionBenchmarkFrame *obstacles, bool use_lidar_cooridinate) {
    ::apollo::common::Header *header = obstacles->mutable_header();
    header->set_timestamp_sec(timestamp);
    header->set_module_name("perception_LidarFrame_benchmark");
    header->set_sequence_num(seq_num);
    header->set_lidar_timestamp(lidar_timestamp);
    header->set_camera_timestamp(0);
    header->set_radar_timestamp(0);

    for (const auto &obj : objects) {
        PerceptionObstacle *obstacle = obstacles->add_perception_obstacle();
        if (!ConvertSegmentedObjectToPb(obj, obstacle,
              pose, use_lidar_cooridinate)) {
            AERROR << "ConvertSegmentedObjectToPb failed, Object:"
                   << obj->ToString();
            return false;
        }
    }
    return true;
}

bool MsgSerializer::ConvertObjectToPb(const base::ObjectPtr &object_ptr,
                                      PerceptionObstacle *pb_msg) {
  if (object_ptr == nullptr || pb_msg == nullptr) {
    return false;
  }

  pb_msg->set_id(object_ptr->track_id);
  pb_msg->set_theta(object_ptr->theta);
  pb_msg->set_confidence(object_ptr->confidence);

  apollo::common::Point3D *obj_center = pb_msg->mutable_position();
  obj_center->set_x(object_ptr->center(0));
  obj_center->set_y(object_ptr->center(1));
  obj_center->set_z(object_ptr->center(2));

  apollo::common::Point3D *obj_velocity = pb_msg->mutable_velocity();
  obj_velocity->set_x(object_ptr->velocity(0));
  obj_velocity->set_y(object_ptr->velocity(1));
  obj_velocity->set_z(object_ptr->velocity(2));

  apollo::common::Point3D *obj_acceleration = pb_msg->mutable_acceleration();
  obj_acceleration->set_x(object_ptr->acceleration(0));
  obj_acceleration->set_y(object_ptr->acceleration(1));
  obj_acceleration->set_z(object_ptr->acceleration(2));

  pb_msg->set_length(object_ptr->size(0));
  pb_msg->set_width(object_ptr->size(1));
  pb_msg->set_height(object_ptr->size(2));

  for (size_t i = 0; i < object_ptr->polygon.size(); ++i) {
    auto &pt = object_ptr->polygon.at(i);
    apollo::common::Point3D *p = pb_msg->add_polygon_point();
    p->set_x(pt.x);
    p->set_y(pt.y);
    p->set_z(pt.z);
  }

  if (FLAGS_obs_benchmark_mode) {
    for (auto &point : object_ptr->lidar_supplement.cloud.points()) {
      pb_msg->add_point_cloud(point.x);
      pb_msg->add_point_cloud(point.y);
      pb_msg->add_point_cloud(point.z);
    }
  }

  apollo::common::Point3D *obj_anchor_point = pb_msg->mutable_anchor_point();
  obj_anchor_point->set_x(object_ptr->anchor_point(0));
  obj_anchor_point->set_y(object_ptr->anchor_point(1));
  obj_anchor_point->set_z(object_ptr->anchor_point(2));

  BBox2D *obj_bbox2d = pb_msg->mutable_bbox2d();
  const base::BBox2DF &box = object_ptr->camera_supplement.box;
  obj_bbox2d->set_xmin(box.xmin);
  obj_bbox2d->set_ymin(box.ymin);
  obj_bbox2d->set_xmax(box.xmax);
  obj_bbox2d->set_ymax(box.ymax);

  for (size_t i = 0; i < 3; i++) {
    for (size_t j = 0; j < 3; j++) {
      pb_msg->add_position_covariance(object_ptr->center_uncertainty(i, j));
      pb_msg->add_velocity_covariance(object_ptr->velocity_uncertainty(i, j));
      pb_msg->add_acceleration_covariance(
          object_ptr->acceleration_uncertainty(i, j));
    }
  }

  pb_msg->set_tracking_time(object_ptr->tracking_time);
  pb_msg->set_type(static_cast<PerceptionObstacle::Type>(object_ptr->type));
  pb_msg->set_sub_type(
      static_cast<PerceptionObstacle::SubType>(object_ptr->sub_type));
  pb_msg->set_timestamp(object_ptr->latest_tracked_time);  // in seconds.
  pb_msg->set_semantic_type(static_cast<PerceptionObstacle::SemanticType>(
      object_ptr->lidar_supplement.semantic_type));

  if (object_ptr->lidar_supplement.height_above_ground != kFloatMax) {
    pb_msg->set_height_above_ground(
        object_ptr->lidar_supplement.height_above_ground);
  } else {
    pb_msg->set_height_above_ground(std::numeric_limits<double>::quiet_NaN());
  }

  if (object_ptr->type == base::ObjectType::VEHICLE) {
    LightStatus *light_status = pb_msg->mutable_light_status();
    const base::CarLight &car_light = object_ptr->car_light;
    light_status->set_brake_visible(car_light.brake_visible);
    light_status->set_brake_switch_on(car_light.brake_switch_on);

    light_status->set_left_turn_visible(car_light.left_turn_visible);
    light_status->set_left_turn_switch_on(car_light.left_turn_switch_on);

    light_status->set_right_turn_visible(car_light.right_turn_visible);
    light_status->set_right_turn_switch_on(car_light.right_turn_switch_on);
  }

  if (FLAGS_obs_save_fusion_supplement &&
      object_ptr->fusion_supplement.on_use) {
    for (const auto &measurement : object_ptr->fusion_supplement.measurements) {
      SensorMeasurement *pb_measurement = pb_msg->add_measurements();
      pb_measurement->set_sensor_id(measurement.sensor_id);
      pb_measurement->set_id(measurement.track_id);

      apollo::common::Point3D *pb_position = pb_measurement->mutable_position();
      pb_position->set_x(measurement.center(0));
      pb_position->set_y(measurement.center(1));
      pb_position->set_z(measurement.center(2));

      pb_measurement->set_theta(measurement.theta);
      pb_measurement->set_length(measurement.size(0));
      pb_measurement->set_width(measurement.size(1));
      pb_measurement->set_height(measurement.size(2));

      apollo::common::Point3D *pb_velocity = pb_measurement->mutable_velocity();
      pb_velocity->set_x(measurement.velocity(0));
      pb_velocity->set_y(measurement.velocity(1));
      pb_velocity->set_z(measurement.velocity(2));

      pb_measurement->set_type(
          static_cast<PerceptionObstacle::Type>(measurement.type));
      // pb_measurement->set_sub_type();
      pb_measurement->set_timestamp(measurement.timestamp);

      BBox2D *pb_box = pb_measurement->mutable_box();
      pb_box->set_xmin(measurement.box.xmin);
      pb_box->set_ymin(measurement.box.ymin);
      pb_box->set_xmax(measurement.box.xmax);
      pb_box->set_ymax(measurement.box.ymax);
    }
  }

// TODO(all): semantic map related, for debugging
//  // record the best prediction trajectory
//  if (object_ptr->feature.get() &&
//      object_ptr->feature->predicted_trajectory_size() > 0) {
//    apollo::perception::DebugMessage *dmsg = pb_msg->mutable_msg();
//    apollo::perception::Trajectory *target_traj = dmsg->add_trajectory();
//    const apollo::prediction::Trajectory &src_traj =
//        object_ptr->feature->predicted_trajectory(0);
//  (*target_traj->mutable_trajectory_point()) = (src_traj.trajectory_point());
//    ADEBUG << "Inserting Trajectores in PB with point size "
//           << src_traj.trajectory_point_size();
//  }

  return true;
}

bool MsgSerializer::ConvertSegmentedObjectToPb(
    const base::ObjectPtr &object_ptr, PerceptionObstacle *pb_msg,
    const Eigen::Affine3d &pose, bool use_lidar_cooridinate) {
    if (object_ptr == nullptr || pb_msg == nullptr) {
        return false;
    }

    pb_msg->set_id(object_ptr->id);
    pb_msg->set_confidence(object_ptr->confidence);

    pb_msg->set_length(object_ptr->size(0));
    pb_msg->set_width(object_ptr->size(1));
    pb_msg->set_height(object_ptr->size(2));

    pb_msg->set_type(static_cast<PerceptionObstacle::Type>(object_ptr->type));
    pb_msg->set_sub_type(static_cast<PerceptionObstacle::SubType>(
        object_ptr->sub_type));
    pb_msg->set_semantic_type(static_cast<PerceptionObstacle::SemanticType>(
      object_ptr->lidar_supplement.semantic_type));

    if (use_lidar_cooridinate) {
        pb_msg->set_theta(object_ptr->theta);
        pb_msg->mutable_position()->set_x(object_ptr->center(0));
        pb_msg->mutable_position()->set_y(object_ptr->center(1));
        pb_msg->mutable_position()->set_z(object_ptr->center(2));

        for (size_t i = 0; i < object_ptr->polygon.size(); ++i) {
            auto &pt = object_ptr->polygon.at(i);
            apollo::common::Point3D *p = pb_msg->add_polygon_point();
            p->set_x(pt.x);
            p->set_y(pt.y);
            p->set_z(pt.z);
        }
        if (FLAGS_obs_benchmark_mode) {
            for (auto &point : object_ptr->lidar_supplement.cloud.points()) {
                pb_msg->add_point_cloud(point.x);
                pb_msg->add_point_cloud(point.y);
                pb_msg->add_point_cloud(point.z);
            }
        }
    } else {
        // theta
        Eigen::Vector3d dir = pose.rotation() *
            object_ptr->direction.cast<double>();
        double world_theta = std::atan2(dir(1), dir(0));
        pb_msg->set_theta(world_theta);

        // center [Attention: Z is CENTER-Z, but tracking is BOTTOM-Z]
        Eigen::Vector3d lidar_center(object_ptr->center(0),
            object_ptr->center(1), object_ptr->center(2));
        lidar_center = pose * lidar_center;
        pb_msg->mutable_position()->set_x(lidar_center[0]);
        pb_msg->mutable_position()->set_y(lidar_center[1]);
        pb_msg->mutable_position()->set_z(lidar_center[2]);

        // polygon
        for (size_t i = 0; i < object_ptr->polygon.size(); ++i) {
            auto &pt = object_ptr->polygon.at(i);
            Eigen::Vector3d trans_point_polygon(pt.x, pt.y, pt.z);
            trans_point_polygon = pose * trans_point_polygon;

            apollo::common::Point3D *p = pb_msg->add_polygon_point();
            p->set_x(trans_point_polygon[0]);
            p->set_y(trans_point_polygon[1]);
            p->set_z(trans_point_polygon[2]);
        }
        if (FLAGS_obs_benchmark_mode) {
            auto cloud = object_ptr->lidar_supplement.cloud_world;
            for (auto &point : cloud.points()) {
                pb_msg->add_point_cloud(point.x);
                pb_msg->add_point_cloud(point.y);
                pb_msg->add_point_cloud(point.z);
            }
        }
    }

    // valued in tracking, this NOT-care
    apollo::common::Point3D *obj_anchor_point = pb_msg->mutable_anchor_point();
    obj_anchor_point->set_x(object_ptr->anchor_point(0));
    obj_anchor_point->set_y(object_ptr->anchor_point(1));
    obj_anchor_point->set_z(object_ptr->anchor_point(2));

    BBox2D *obj_bbox2d = pb_msg->mutable_bbox2d();
    const base::BBox2DF &box = object_ptr->camera_supplement.box;
    obj_bbox2d->set_xmin(box.xmin);
    obj_bbox2d->set_ymin(box.ymin);
    obj_bbox2d->set_xmax(box.xmax);
    obj_bbox2d->set_ymax(box.ymax);

    if (object_ptr->lidar_supplement.height_above_ground != kFloatMax) {
        pb_msg->set_height_above_ground(
            object_ptr->lidar_supplement.height_above_ground);
    } else {
        pb_msg->set_height_above_ground(
            std::numeric_limits<double>::quiet_NaN());
    }

    if (object_ptr->type == base::ObjectType::VEHICLE) {
        LightStatus *light_status = pb_msg->mutable_light_status();
        const base::CarLight &car_light = object_ptr->car_light;
        light_status->set_brake_visible(car_light.brake_visible);
        light_status->set_brake_switch_on(car_light.brake_switch_on);

        light_status->set_left_turn_visible(car_light.left_turn_visible);
        light_status->set_left_turn_switch_on(car_light.left_turn_switch_on);

        light_status->set_right_turn_visible(car_light.right_turn_visible);
        light_status->set_right_turn_switch_on(car_light.right_turn_switch_on);
    }
    return true;
}

}  // namespace onboard
}  // namespace perception
}  // namespace apollo
