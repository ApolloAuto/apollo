/******************************************************************************
 * Copyright 2021 The Apollo Authors. All Rights Reserved.
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
#include <string>

#include "modules/common_msgs/sensor_msgs/pointcloud.pb.h"

#include "cyber/common/macros.h"
#include "modules/perception/common/lib/interface/base_init_options.h"
#include "modules/perception/common/lib/registerer/registerer.h"
#include "modules/perception/common/lidar/common/lidar_frame.h"

namespace apollo {
namespace perception {
namespace lidar {

using apollo::perception::BaseInitOptions;

struct PointCloudPreprocessorInitOptions : public BaseInitOptions {
  std::string sensor_name = "velodyne64";
};

struct PointCloudPreprocessorOptions {
  Eigen::Affine3d sensor2novatel_extrinsics;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

class BasePointCloudPreprocessor {
 public:
  BasePointCloudPreprocessor() = default;

  virtual ~BasePointCloudPreprocessor() = default;
  /**
   * @brief Init pointcloud preprocessor
   *
   * @param options
   * @return true
   * @return false
   */
  virtual bool Init(const PointCloudPreprocessorInitOptions& options =
                        PointCloudPreprocessorInitOptions()) = 0;

  /**
   * @brief Preprocess point cloud
   *
   * @param options
   * @param message Point cloud message
   * @param frame Location of pointcloud data write to
   * @return true
   * @return false
   */
  virtual bool Preprocess(
      const PointCloudPreprocessorOptions& options,
      const std::shared_ptr<apollo::drivers::PointCloud const>& message,
      LidarFrame* frame) const = 0;
  /**
   * @brief Preprocess point cloud
   *
   * @param options
   * @param frame
   * @return true
   * @return false
   */
  virtual bool Preprocess(const PointCloudPreprocessorOptions& options,
                          LidarFrame* frame) const = 0;
  /**
   * @brief Get class name
   *
   * @return std::string
   */
  virtual std::string Name() const = 0;

 private:
  DISALLOW_COPY_AND_ASSIGN(BasePointCloudPreprocessor);
};  // class BasePointCloudPreprocessor

PERCEPTION_REGISTER_REGISTERER(BasePointCloudPreprocessor);
#define PERCEPTION_REGISTER_POINTCLOUDPREPROCESSOR(name) \
  PERCEPTION_REGISTER_CLASS(BasePointCloudPreprocessor, name)

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
