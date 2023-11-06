/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#include "modules/common_msgs/perception_msgs/perception_obstacle.pb.h"

#include "cyber/cyber.h"
#include "cyber/time/clock.h"
#include "modules/perception/common/onboard/inner_component_messages/camera_detection_component_messages.h"
#include "modules/perception/common/onboard/inner_component_messages/inner_component_messages.h"
#include "modules/perception/common/onboard/inner_component_messages/lidar_inner_component_messages.h"
#include "modules/perception/msg_adapter/convert/common.h"

namespace apollo {
namespace perception {

bool ConvertCameraFrame2Obstacles(
    const std::shared_ptr<onboard::CameraFrame> &frame,
    PerceptionObstacles *obstacles) {
  static uint32_t seq_num = 0;
  auto header = obstacles->mutable_header();
  header->set_timestamp_sec(cyber::Clock::NowInSeconds());
  header->set_module_name("perception_camera");
  header->set_sequence_num(seq_num++);
  // nanosecond
  header->set_lidar_timestamp(static_cast<uint64_t>(frame->timestamp * 1e9));
  header->set_camera_timestamp(static_cast<uint64_t>(frame->timestamp * 1e9));

  obstacles->set_error_code(apollo::common::OK);

  for (const auto &obj : frame->detected_objects) {
    PerceptionObstacle *obstacle = obstacles->add_perception_obstacle();
    if (!ConvertObjectToPb(obj, obstacle)) {
      AERROR << "ConvertObjectToPb failed, Object:" << obj->ToString();
      return false;
    }
  }
  return true;
}

bool ConvertSensorFrameMessage2Obstacles(
    const std::shared_ptr<onboard::SensorFrameMessage> &msg,
    PerceptionObstacles *obstacles) {
  auto header = obstacles->mutable_header();
  header->set_timestamp_sec(cyber::Clock::NowInSeconds());
  header->set_module_name("perception_obstacle");
  header->set_sequence_num(msg->seq_num_);
  header->set_lidar_timestamp(msg->lidar_timestamp_);
  header->set_camera_timestamp(0);
  header->set_radar_timestamp(0);

  obstacles->set_error_code(apollo::common::ErrorCode::OK);
  if (msg != nullptr && msg->frame_ != nullptr) {
    for (const auto &obj : msg->frame_->objects) {
      PerceptionObstacle *obstacle = obstacles->add_perception_obstacle();
      if (!ConvertObjectToPb(obj, obstacle)) {
        AERROR << "ConvertObjectToPb failed, Object:" << obj->ToString();
        return false;
      }
    }
  }
  return true;
}

bool ConvertLidarFrameMessage2Obstacles(
    const std::shared_ptr<onboard::LidarFrameMessage> &msg,
    PerceptionObstacles *obstacles) {
  static uint32_t seq_num = 0;
  auto header = obstacles->mutable_header();
  header->set_timestamp_sec(cyber::Clock::NowInSeconds());
  header->set_module_name("perception_lidar");
  header->set_sequence_num(seq_num++);
  header->set_lidar_timestamp(msg->lidar_timestamp_);
  header->set_camera_timestamp(0);
  header->set_radar_timestamp(0);

  obstacles->set_error_code(apollo::common::OK);

  int id = 0;
  for (const auto &obj : msg->lidar_frame_.get()->segmented_objects) {
    Eigen::Vector3d trans_point(obj->center(0), obj->center(1), obj->center(2));
    trans_point = msg->lidar_frame_.get()->lidar2world_pose * trans_point;
    obj->center(0) = trans_point[0];
    obj->center(1) = trans_point[1];
    obj->center(2) = trans_point[2];

    for (size_t i = 0; i < obj->polygon.size(); ++i) {
      auto &pt = obj->polygon[i];
      Eigen::Vector3d trans_point_polygon(pt.x, pt.y, pt.z);
      trans_point_polygon =
          msg->lidar_frame_.get()->lidar2world_pose * trans_point_polygon;
      pt.x = trans_point_polygon[0];
      pt.y = trans_point_polygon[1];
      pt.z = trans_point_polygon[2];
    }

    base::PointDCloud& cloud_world = (obj->lidar_supplement).cloud_world;
    cloud_world.clear();
    cloud_world.resize(obj->lidar_supplement.cloud.size());
    for (size_t i = 0; i < obj->lidar_supplement.cloud.size(); ++i) {
      Eigen::Vector3d pt(obj->lidar_supplement.cloud[i].x,
                         obj->lidar_supplement.cloud[i].y,
                         obj->lidar_supplement.cloud[i].z);
      Eigen::Vector3d pt_world = msg->lidar_frame_.get()->lidar2world_pose * pt;
      cloud_world[i].x = pt_world(0);
      cloud_world[i].y = pt_world(1);
      cloud_world[i].z = pt_world(2);
      cloud_world[i].intensity = obj->lidar_supplement.cloud[i].intensity;
    }

    for (size_t i = 0; i < obj->lidar_supplement.cloud.size(); ++i) {
      cloud_world.SetPointHeight(i,
                                 obj->lidar_supplement.cloud.points_height(i));
    }

    Eigen::Vector3d trans_anchor_point(obj->anchor_point(0),
                                       obj->anchor_point(1),
                                       obj->anchor_point(2));
    trans_anchor_point =
        msg->lidar_frame_.get()->lidar2world_pose * trans_anchor_point;
    obj->anchor_point(0) = trans_anchor_point[0];
    obj->anchor_point(1) = trans_anchor_point[1];
    obj->anchor_point(2) = trans_anchor_point[2];

    obj->track_id = id++;
  }

  for (const auto &obj : msg->lidar_frame_.get()->segmented_objects) {
    PerceptionObstacle *obstacle = obstacles->add_perception_obstacle();
    if (!ConvertObjectToPb(obj, obstacle)) {
      AERROR << "ConvertObjectToPb failed, Object:" << obj->ToString();
      return false;
    }
  }
  return true;
}

}  // namespace perception
}  // namespace apollo
