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

#pragma once

#include <limits>
#include <string>
#include <vector>

#include "modules/perception/lidar_cpdet_detection/down_sample_bank/pcl_method/proto/pcl_method.pb.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/perception/common/base/point_cloud.h"
#include "modules/perception/common/interface/base_down_sample.h"

namespace apollo {
namespace perception {
namespace lidar {

class PclPCDownSample : public BaseDownSample {
 public:
  PclPCDownSample() = default;
  virtual ~PclPCDownSample() = default;

  /**
   * @brief Init of PclPCDownSample object
   *
   * @param options object down sample options
   * @return true
   * @return false
   */
  bool Init(
      const DownSampleInitOptions& options = DownSampleInitOptions()) override;

  /**
   * @brief Down sample pointcloud
   *
   * @param options down sample options
   * @param cloud_ptr point cloud to process
   * @return true
   * @return false
   */
  bool Process(const DownSampleOptions& options,
               base::PointFCloudPtr& cloud_ptr) override;

  /**
   * @brief Name of PclPCDownSample object
   *
   * @return std::string name
   */
  std::string Name() const override { return "PclPCDownSample"; }

 private:
  double downsample_voxel_size_x_ = 0.09;
  double downsample_voxel_size_y_ = 0.09;
  double downsample_voxel_size_z_ = 0.09;
};  // class PclPCDownSample

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::perception::lidar::PclPCDownSample,
                                     BaseDownSample)

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
