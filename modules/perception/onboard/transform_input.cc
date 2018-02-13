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
#include "modules/perception/onboard/transform_input.h"

#include <string>

#include "eigen_conversions/eigen_msg.h"
#include "ros/include/ros/ros.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/perception/common/perception_gflags.h"

namespace apollo {
namespace perception {

bool GetVelodyneTrans(const double query_time, Eigen::Matrix4d* trans) {
  if (!trans) {
    AERROR << "failed to get trans, the trans ptr can not be NULL";
    return false;
  }

  ros::Time query_stamp(query_time);
  const auto& tf2_buffer = common::adapter::AdapterManager::Tf2Buffer();

  const double kTf2BuffSize = FLAGS_tf2_buff_in_ms / 1000.0;
  std::string err_msg;
  if (!tf2_buffer.canTransform(FLAGS_lidar_tf2_frame_id,
                               FLAGS_lidar_tf2_child_frame_id, query_stamp,
                               ros::Duration(kTf2BuffSize), &err_msg)) {
    AERROR << "Cannot transform frame: " << FLAGS_lidar_tf2_frame_id
           << " to frame " << FLAGS_lidar_tf2_child_frame_id
           << " , err: " << err_msg
           << ". Frames: " << tf2_buffer.allFramesAsString();
    return false;
  }

  geometry_msgs::TransformStamped transform_stamped;
  try {
    transform_stamped = tf2_buffer.lookupTransform(
        FLAGS_lidar_tf2_frame_id, FLAGS_lidar_tf2_child_frame_id, query_stamp);
  } catch (tf2::TransformException& ex) {
    AERROR << "Exception: " << ex.what();
    return false;
  }
  Eigen::Affine3d affine_lidar_3d;
  tf::transformMsgToEigen(transform_stamped.transform, affine_lidar_3d);
  Eigen::Matrix4d lidar2novatel_trans = affine_lidar_3d.matrix();
  ADEBUG << "get " << FLAGS_lidar_tf2_frame_id << " to "
         << FLAGS_lidar_tf2_child_frame_id << " trans: " << lidar2novatel_trans;

  if (!tf2_buffer.canTransform(FLAGS_localization_tf2_frame_id,
                               FLAGS_localization_tf2_child_frame_id,
                               query_stamp, ros::Duration(kTf2BuffSize),
                               &err_msg)) {
    AERROR << "Cannot transform frame: " << FLAGS_localization_tf2_frame_id
           << " to frame " << FLAGS_localization_tf2_child_frame_id
           << " , err: " << err_msg
           << ". Frames: " << tf2_buffer.allFramesAsString();
    return false;
  }
  try {
    transform_stamped = tf2_buffer.lookupTransform(
        FLAGS_localization_tf2_frame_id, FLAGS_localization_tf2_child_frame_id,
        query_stamp);
  } catch (tf2::TransformException& ex) {
    AERROR << "Exception: " << ex.what();
    return false;
  }
  Eigen::Affine3d affine_localization_3d;
  tf::transformMsgToEigen(transform_stamped.transform, affine_localization_3d);
  Eigen::Matrix4d novatel2world_trans = affine_localization_3d.matrix();

  *trans = novatel2world_trans * lidar2novatel_trans;
  ADEBUG << "get " << FLAGS_lidar_tf2_frame_id << " to "
         << FLAGS_localization_tf2_child_frame_id << " trans: " << *trans;
  return true;
}

bool GetRadarTrans(const double query_time, Eigen::Matrix4d* trans) {
  if (!trans) {
    AERROR << "failed to get trans, the trans ptr can not be NULL";
    return false;
  }

  ros::Time query_stamp(query_time);
  const auto& tf2_buffer = common::adapter::AdapterManager::Tf2Buffer();

  const double kTf2BuffSize = FLAGS_tf2_buff_in_ms / 1000.0;
  std::string err_msg;
  if (!tf2_buffer.canTransform(FLAGS_radar_tf2_frame_id,
                               FLAGS_radar_tf2_child_frame_id, query_stamp,
                               ros::Duration(kTf2BuffSize), &err_msg)) {
    AERROR << "Cannot transform frame: " << FLAGS_radar_tf2_frame_id
           << " to frame " << FLAGS_radar_tf2_child_frame_id
           << " , err: " << err_msg
           << ". Frames: " << tf2_buffer.allFramesAsString();
    return false;
  }

  geometry_msgs::TransformStamped transform_stamped;
  try {
    transform_stamped = tf2_buffer.lookupTransform(
        FLAGS_radar_tf2_frame_id, FLAGS_radar_tf2_child_frame_id, query_stamp);
  } catch (tf2::TransformException& ex) {
    AERROR << "Exception: " << ex.what();
    return false;
  }
  Eigen::Affine3d affine_radar_3d;
  tf::transformMsgToEigen(transform_stamped.transform, affine_radar_3d);
  Eigen::Matrix4d radar2novatel_trans = affine_radar_3d.matrix();
  ADEBUG << "get " << FLAGS_radar_tf2_frame_id << " to "
         << FLAGS_radar_tf2_child_frame_id << " trans: " << radar2novatel_trans;

  if (!tf2_buffer.canTransform(FLAGS_localization_tf2_frame_id,
                               FLAGS_localization_tf2_child_frame_id,
                               query_stamp, ros::Duration(kTf2BuffSize),
                               &err_msg)) {
    AERROR << "Cannot transform frame: " << FLAGS_localization_tf2_frame_id
           << " to frame " << FLAGS_localization_tf2_child_frame_id
           << " , err: " << err_msg
           << ". Frames: " << tf2_buffer.allFramesAsString();
    return false;
  }
  try {
    transform_stamped = tf2_buffer.lookupTransform(
        FLAGS_localization_tf2_frame_id, FLAGS_localization_tf2_child_frame_id,
        query_stamp);
  } catch (tf2::TransformException& ex) {
    AERROR << "Exception: " << ex.what();
    return false;
  }
  Eigen::Affine3d affine_localization_3d;
  tf::transformMsgToEigen(transform_stamped.transform, affine_localization_3d);
  Eigen::Matrix4d novatel2world_trans = affine_localization_3d.matrix();

  *trans = novatel2world_trans * radar2novatel_trans;
  ADEBUG << "get " << FLAGS_radar_tf2_frame_id << " to "
         << FLAGS_localization_tf2_child_frame_id << " trans: " << *trans;
  return true;
}

}  // namespace perception
}  // namespace apollo
