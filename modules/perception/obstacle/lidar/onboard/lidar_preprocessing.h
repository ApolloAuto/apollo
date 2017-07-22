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

#ifndef MODULES_PERCEPTION_OBSTACLE_ONBOARD_LIDAR_PREPROCESSING_H_
#define MODULES_PERCEPTION_OBSTACLE_ONBOARD_LIDAR_PREPROCESSING_H_

#include <memory>
#include <mutex>
#include <sensor_msgs/PointCloud2.h>

#include "modules/perception/obstacle/lidar/onboard/lidar_predetection_data.h"
#include "modules/perception/obstacle/lidar/onboard/component.h"
#include "modules/perception/lib/pcl_util/pcl_types.h"
#include "modules/perception/obstacle/lidar/interface/base_roi_filter.h"

namespace apollo {
namespace perception {

class LidarPreprocessing
    : public Component {
 public:
  LidarPreprocessing() = default;
  ~LidarPreprocessing() = default;

  bool Init() override;

  bool Proc(const ::sensor_msgs::PointCloud2& message, LidarPredetectionData* data);
  
  std::string Name() const override;

 protected:
  bool InitConfig() override;
  bool InitAlgorithmPlugin() override;

  void trans_pointcloud_to_pcl(
        const ::sensor_msgs::PointCloud2& in_msg,
        pcl_util::PointCloudPtr* out_cloud);
  bool GetVelodyneWorldTrans(double timestamp, Eigen::Matrix4d* trans);

 private:

  DISALLOW_COPY_AND_ASSIGN(LidarPreprocessing); 
};


}  // namespace perception
}  // namespace apollo

#endif  // APOLLO_PERCEPTION_OBSTACLE_ONBOARD_LIDAR_PREPROCESSING_H_
