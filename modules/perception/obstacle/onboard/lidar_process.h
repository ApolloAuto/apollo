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

#ifndef MODEULES_PERCEPTION_OBSTACLE_ONBOARD_LIDAR_PROCESS_H_
#define MODEULES_PERCEPTION_OBSTACLE_ONBOARD_LIDAR_PROCESS_H_

#include <sensor_msgs/PointCloud2.h>
#include "modules/perception/lib/pcl_util/pcl_types.h"
#include "modules/perception/obstacle/base/object.h"
#include "modules/perception/obstacle/lidar/interface/base_object_builder.h"
#include "modules/perception/obstacle/lidar/interface/base_roi_filter.h"
#include "modules/perception/obstacle/lidar/interface/base_segmentation.h"
#include "modules/perception/obstacle/lidar/interface/base_tracker.h"
#include "modules/perception/obstacle/onboard/hdmap_input.h"
#include "modules/perception/proto/perception_obstacle.pb.h"

namespace apollo {
namespace perception {

class LidarProcess {
 public:
  LidarProcess() = default;
  ~LidarProcess() = default;

  bool Init();
  bool IsInit() {
    return inited_;
  }
  bool Process(const sensor_msgs::PointCloud2& message);

  bool GeneratePbMsg(PerceptionObstacles* obstacles);

 private:
  bool InitFrameDependence();
  bool InitAlgorithmPlugin();

  void TransPointCloudToPCL(const sensor_msgs::PointCloud2& in_msg,
                            pcl_util::PointCloudPtr* out_cloud);
  bool GetVelodyneTrans(const double query_time, Eigen::Matrix4d* trans);

  bool inited_ = false;
  size_t seq_num_ = 0;
  double timestamp_;
  apollo::common::ErrorCode error_code_ = apollo::common::OK;
  std::vector<ObjectPtr> objects_;
  HDMapInput* hdmap_input_ = NULL;
  std::unique_ptr<BaseROIFilter> roi_filter_;
  std::unique_ptr<BaseSegmentation> segmentor_;
  std::unique_ptr<BaseObjectBuilder> object_builder_;
  std::unique_ptr<BaseTracker> tracker_;
};

}  // namespace perception
}  // namespace apollo

#endif  // MODEULES_PERCEPTION_OBSTACLE_ONBOARD_LIDAR_PROCESS_H_
