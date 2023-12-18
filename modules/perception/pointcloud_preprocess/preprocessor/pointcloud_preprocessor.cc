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

#include "modules/perception/pointcloud_preprocess/preprocessor/pointcloud_preprocessor.h"

#include <limits>

#include "modules/perception/pointcloud_preprocess/preprocessor/proto/pointcloud_preprocessor_config.pb.h"

#include "cyber/common/file.h"
#include "modules/perception/common/util.h"
#include "modules/perception/common/base/object_pool_types.h"
#include "modules/perception/common/lidar/common/lidar_log.h"

namespace apollo {
namespace perception {
namespace lidar {

const float PointCloudPreprocessor::kPointInfThreshold = 1e3;

bool PointCloudPreprocessor::Init(
    const PointCloudPreprocessorInitOptions& options) {
  std::string config_file =
      GetConfigFile(options.config_path, options.config_file);
  PointCloudPreprocessorConfig config;
  ACHECK(apollo::cyber::common::GetProtoFromFile(config_file, &config));
  filter_naninf_points_ = config.filter_naninf_points();
  filter_nearby_box_points_ = config.filter_nearby_box_points();
  box_forward_x_ = config.box_forward_x();
  box_backward_x_ = config.box_backward_x();
  box_forward_y_ = config.box_forward_y();
  box_backward_y_ = config.box_backward_y();
  filter_high_z_points_ = config.filter_high_z_points();
  z_threshold_ = config.z_threshold();
  return true;
}

bool PointCloudPreprocessor::Preprocess(
    const PointCloudPreprocessorOptions& options,
    const std::shared_ptr<apollo::drivers::PointCloud const>& message,
    LidarFrame* frame) const {
  if (frame == nullptr) {
    return false;
  }
  if (frame->cloud == nullptr) {
    frame->cloud = base::PointFCloudPool::Instance().Get();
  }
  if (frame->world_cloud == nullptr) {
    frame->world_cloud = base::PointDCloudPool::Instance().Get();
  }

  frame->cloud->set_timestamp(message->measurement_time());
  if (message->point_size() > 0) {
    frame->cloud->reserve(message->point_size());
    base::PointF point;
    for (int i = 0; i < message->point_size(); ++i) {
      const apollo::drivers::PointXYZIT& pt = message->point(i);
      if (filter_naninf_points_) {
        if (std::isnan(pt.x()) || std::isnan(pt.y()) || std::isnan(pt.z())) {
          continue;
        }
        if (fabs(pt.x()) > kPointInfThreshold ||
            fabs(pt.y()) > kPointInfThreshold ||
            fabs(pt.z()) > kPointInfThreshold) {
          continue;
        }
      }
      Eigen::Vector3d vec3d_lidar(pt.x(), pt.y(), pt.z());
      // Eigen::Vector3d vec3d_novatel =
      //     options.sensor2novatel_extrinsics * vec3d_lidar;
      Eigen::Vector3d vec3d_novatel = vec3d_lidar;
      if (filter_nearby_box_points_ && vec3d_novatel[0] < box_forward_x_ &&
          vec3d_novatel[0] > box_backward_x_ &&
          vec3d_novatel[1] < box_forward_y_ &&
          vec3d_novatel[1] > box_backward_y_) {
        continue;
      }
      if (filter_high_z_points_ && pt.z() > z_threshold_) {
        continue;
      }
      point.x = pt.x();
      point.y = pt.y();
      point.z = pt.z();
      point.intensity = static_cast<float>(pt.intensity());
      frame->cloud->push_back(point, static_cast<double>(pt.timestamp()) * 1e-9,
                              std::numeric_limits<float>::max(), i, 0);
    }
    TransformCloud(frame->cloud, frame->lidar2world_pose, frame->world_cloud);
  }

  return true;
}

bool PointCloudPreprocessor::Preprocess(
    const PointCloudPreprocessorOptions& options, LidarFrame* frame) const {
  if (frame == nullptr || frame->cloud == nullptr) {
    return false;
  }
  if (frame->world_cloud == nullptr) {
    frame->world_cloud = base::PointDCloudPool::Instance().Get();
  }
  if (frame->cloud->size() > 0) {
    size_t size = frame->cloud->size();
    size_t i = 0;
    while (i < size) {
      auto& pt = frame->cloud->at(i);
      if (filter_naninf_points_) {
        if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z)) {
          frame->cloud->SwapPoint(i, size--);
          continue;
        }
        if (fabs(pt.x) > kPointInfThreshold ||
            fabs(pt.y) > kPointInfThreshold ||
            fabs(pt.z) > kPointInfThreshold) {
          frame->cloud->SwapPoint(i, size--);
          continue;
        }
      }
      Eigen::Vector3d vec3d_lidar(pt.x, pt.y, pt.z);
      // Eigen::Vector3d vec3d_novatel =
      //     options.sensor2novatel_extrinsics * vec3d_lidar;
      Eigen::Vector3d vec3d_novatel = vec3d_lidar;
      if (filter_nearby_box_points_ && vec3d_novatel[0] < box_forward_x_ &&
          vec3d_novatel[0] > box_backward_x_ &&
          vec3d_novatel[1] < box_forward_y_ &&
          vec3d_novatel[1] > box_backward_y_) {
        frame->cloud->SwapPoint(i, size--);
        continue;
      }
      if (filter_high_z_points_ && pt.z > z_threshold_) {
        frame->cloud->SwapPoint(i, size--);
        continue;
      }
      ++i;
    }
    frame->cloud->resize(i);
    TransformCloud(frame->cloud, frame->lidar2world_pose, frame->world_cloud);
    AINFO << "Preprocessor filter points: " << size << " to " << i;
  }
  return true;
}

bool PointCloudPreprocessor::TransformCloud(
    const base::PointFCloudPtr& local_cloud, const Eigen::Affine3d& pose,
    base::PointDCloudPtr world_cloud) const {
  if (local_cloud == nullptr) {
    return false;
  }
  world_cloud->clear();
  world_cloud->reserve(local_cloud->size());
  for (size_t i = 0; i < local_cloud->size(); ++i) {
    auto& pt = local_cloud->at(i);
    Eigen::Vector3d trans_point(pt.x, pt.y, pt.z);
    trans_point = pose * trans_point;
    base::PointD world_point;
    world_point.x = trans_point(0);
    world_point.y = trans_point(1);
    world_point.z = trans_point(2);
    world_point.intensity = pt.intensity;
    world_cloud->push_back(world_point, local_cloud->points_timestamp(i),
                           std::numeric_limits<float>::max(),
                           local_cloud->points_beam_id()[i], 0);
  }
  return true;
}

PERCEPTION_REGISTER_POINTCLOUDPREPROCESSOR(PointCloudPreprocessor);

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
