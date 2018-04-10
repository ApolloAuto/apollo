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

#ifndef MODULES_PERCEPTION_OBSTACLE_ONBORAD_LIDAR_PROCESS_SUBNODE_H_
#define MODULES_PERCEPTION_OBSTACLE_ONBORAD_LIDAR_PROCESS_SUBNODE_H_

#include <memory>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "gtest/gtest_prod.h"
#include "sensor_msgs/PointCloud2.h"

#include "modules/perception/proto/perception_obstacle.pb.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/perception/common/pcl_types.h"
#include "modules/perception/common/sequence_type_fuser/base_type_fuser.h"
#include "modules/perception/obstacle/base/object.h"
#include "modules/perception/obstacle/lidar/interface/base_object_builder.h"
#include "modules/perception/obstacle/lidar/interface/base_object_filter.h"
#include "modules/perception/obstacle/lidar/interface/base_roi_filter.h"
#include "modules/perception/obstacle/lidar/interface/base_segmentation.h"
#include "modules/perception/obstacle/lidar/interface/base_tracker.h"
#include "modules/perception/obstacle/lidar/visualizer/opengl_visualizer/frame_content.h"
#include "modules/perception/obstacle/lidar/visualizer/opengl_visualizer/opengl_visualizer.h"
#include "modules/perception/obstacle/onboard/hdmap_input.h"
#include "modules/perception/obstacle/onboard/object_shared_data.h"
#include "modules/perception/onboard/subnode.h"

namespace apollo {
namespace perception {

class LidarProcessSubnode : public Subnode {
 public:
  LidarProcessSubnode() = default;
  ~LidarProcessSubnode() = default;

  apollo::common::Status ProcEvents() override {
    return apollo::common::Status::OK();
  }

 private:
  bool InitInternal() override;

  void OnPointCloud(const sensor_msgs::PointCloud2& message);

  pcl_util::PointIndicesPtr GetROIIndices() { return roi_indices_; }

  void RegistAllAlgorithm();
  bool InitFrameDependence();
  bool InitAlgorithmPlugin();

  void TransPointCloudToPCL(const sensor_msgs::PointCloud2& in_msg,
                            pcl_util::PointCloudPtr* out_cloud);

  void PublishDataAndEvent(double timestamp,
                           const SharedDataPtr<SensorObjects>& data);

  bool inited_ = false;
  double timestamp_ = 0.0;
  SeqId seq_num_ = 0;
  common::ErrorCode error_code_ = common::OK;
  LidarObjectData* processing_data_ = nullptr;
  std::string device_id_;

  HDMapInput* hdmap_input_ = nullptr;
  std::unique_ptr<BaseROIFilter> roi_filter_;
  std::unique_ptr<BaseSegmentation> segmentor_;
  std::unique_ptr<BaseObjectFilter> object_filter_;
  std::unique_ptr<BaseObjectBuilder> object_builder_;
  std::unique_ptr<BaseTracker> tracker_;
  std::unique_ptr<BaseTypeFuser> type_fuser_;
  pcl_util::PointIndicesPtr roi_indices_;
};

REGISTER_SUBNODE(LidarProcessSubnode);

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_ONBORAD_LIDAR_PROCESS_SUBNODE_H_
