/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/lidar_detection/down_sample_bank/pcl_down_sample/pcl_down_sample.h"

#include <algorithm>

#include "modules/perception/common/lidar/common/pcl_util.h"
#include "modules/perception/common/util.h"

namespace apollo {
namespace perception {
namespace lidar {

bool PclDownSample::Init(const DownSampleInitOptions& options) {
  std::string config_file =
      GetConfigFile(options.config_path, options.config_file);
  PclDownSampleConfig config;
  ACHECK(cyber::common::GetProtoFromFile(config_file, &config));
  downsample_voxel_size_x_ = config.downsample_voxel_size_x();
  downsample_voxel_size_y_ = config.downsample_voxel_size_y();
  downsample_voxel_size_z_ = config.downsample_voxel_size_z();
  AINFO << "PclDownSample plugin init success.";
  return true;
}

bool PclDownSample::Process(const DownSampleOptions& options,
                            base::PointFCloudPtr& cloud_ptr) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud_ptr(
      new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(
      new pcl::PointCloud<pcl::PointXYZI>());

  TransformToPCLXYZI(*cloud_ptr, pcl_cloud_ptr);
  DownSampleCloudByVoxelGrid(pcl_cloud_ptr, filtered_cloud_ptr,
                             downsample_voxel_size_x_, downsample_voxel_size_y_,
                             downsample_voxel_size_z_);
  // transform pcl point cloud to apollo point cloud
  base::PointFCloudPtr downsample_voxel_cloud_ptr(new base::PointFCloud());
  TransformFromPCLXYZI(filtered_cloud_ptr, downsample_voxel_cloud_ptr);
  cloud_ptr = downsample_voxel_cloud_ptr;
  return true;
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
