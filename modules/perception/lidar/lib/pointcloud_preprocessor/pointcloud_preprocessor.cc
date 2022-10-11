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
#include "modules/perception/lidar/lib/pointcloud_preprocessor/pointcloud_preprocessor.h"

#include <limits>
#include <unordered_map>

#include "cyber/common/file.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/perception/base/object_pool_types.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/lidar/common/lidar_log.h"

namespace apollo {
namespace perception {
namespace lidar {

using cyber::common::GetAbsolutePath;

const float PointCloudPreprocessor::kPointInfThreshold = 1e3;

bool PointCloudPreprocessor::Init(
    const PointCloudPreprocessorInitOptions& options) {
  auto config_manager = lib::ConfigManager::Instance();
  const lib::ModelConfig* model_config = nullptr;
  ACHECK(config_manager->GetModelConfig(Name(), &model_config));
  const std::string work_root = config_manager->work_root();
  std::string config_file;
  std::string root_path;
  ACHECK(model_config->get_value("root_path", &root_path));
  config_file = GetAbsolutePath(work_root, root_path);
  config_file = GetAbsolutePath(config_file, options.sensor_name);
  config_file = GetAbsolutePath(config_file, "pointcloud_preprocessor.conf");
  PointcloudPreprocessorConfig config;
  ACHECK(apollo::cyber::common::GetProtoFromFile(config_file, &config));
  filter_naninf_points_ = config.filter_naninf_points();
  filter_nearby_box_points_ = config.filter_nearby_box_points();
  box_forward_x_ = config.box_forward_x();
  box_backward_x_ = config.box_backward_x();
  box_forward_y_ = config.box_forward_y();
  box_backward_y_ = config.box_backward_y();
  /*const auto &vehicle_param =
    common::VehicleConfigHelper::GetConfig().vehicle_param();
  box_forward_x_ = static_cast<float>(vehicle_param.right_edge_to_center());
  box_backward_x_ = static_cast<float>(-vehicle_param.left_edge_to_center());
  box_forward_y_ = static_cast<float>(vehicle_param.front_edge_to_center());
  box_backward_y_ = static_cast<float>(-vehicle_param.back_edge_to_center());*/
  filter_high_z_points_ = config.filter_high_z_points();
  z_threshold_ = config.z_threshold();
  return true;
}

bool PointCloudPreprocessor::Init(const StageConfig& stage_config) {
  if (!Initialize(stage_config)) {
    return false;
  }

  ACHECK(stage_config.has_pointcloud_preprocessor_config());
  pointcloud_preprocessor_config_ =
      stage_config.pointcloud_preprocessor_config();

  filter_naninf_points_ =
      pointcloud_preprocessor_config_.filter_naninf_points();
  filter_nearby_box_points_ =
      pointcloud_preprocessor_config_.filter_nearby_box_points();
  box_forward_x_  = pointcloud_preprocessor_config_.box_forward_x();
  box_backward_x_ = pointcloud_preprocessor_config_.box_backward_x();
  box_forward_y_  = pointcloud_preprocessor_config_.box_forward_y();
  box_backward_y_ = pointcloud_preprocessor_config_.box_backward_y();
  filter_high_z_points_ =
      pointcloud_preprocessor_config_.filter_high_z_points();
  z_threshold_    = pointcloud_preprocessor_config_.z_threshold();
  return true;
}

// Process作用： 1、代替原有的Process逻辑(封装成为InnerProcess) ，
// 2、循环调用不同task的Process函数(如果有task)
bool PointCloudPreprocessor::Process(DataFrame* data_frame) {
  if (data_frame == nullptr) return false;

  LidarFrame* lidar_frame = data_frame->lidar_frame;
  if (lidar_frame == nullptr) return false;

  PointCloudPreprocessorOptions options;
  options.sensor2novatel_extrinsics = lidar_frame->lidar2novatel_extrinsics;
  bool result = Preprocess(options, lidar_frame);
  return result;
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
      Eigen::Vector3d vec3d_novatel =
          options.sensor2novatel_extrinsics * vec3d_lidar;
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
      Eigen::Vector3d vec3d_novatel =
          options.sensor2novatel_extrinsics * vec3d_lidar;
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
    AINFO << "Preprocessor filter points: "
          << frame->cloud->size() << " to " << size;
    frame->cloud->resize(size);
    TransformCloud(frame->cloud, frame->lidar2world_pose, frame->world_cloud);
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
