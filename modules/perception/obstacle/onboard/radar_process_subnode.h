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

#ifndef MODULES_PERCEPTION_OBSTACLE_ONBOARD_SUBNODE_H_
#define MODULES_PERCEPTION_OBSTACLE_ONBOARD_SUBNODE_H_
#include <boost/circular_buffer.hpp>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "gtest/gtest_prod.h"
#include "sensor_msgs/PointCloud2.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/perception/common/pcl_types.h"
#include "modules/perception/obstacle/base/object.h"
#include "modules/perception/obstacle/common/pose_util.h"
#include "modules/perception/obstacle/lidar/interface/base_roi_filter.h"
#include "modules/perception/obstacle/lidar/roi_filter/hdmap_roi_filter/hdmap_roi_filter.h"
#include "modules/perception/obstacle/lidar/visualizer/opengl_visualizer/frame_content.h"
#include "modules/perception/obstacle/lidar/visualizer/opengl_visualizer/opengl_visualizer.h"
#include "modules/perception/obstacle/onboard/hdmap_input.h"
#include "modules/perception/obstacle/onboard/object_shared_data.h"
#include "modules/perception/obstacle/radar/interface/base_radar_detector.h"
#include "modules/perception/obstacle/radar/modest/conti_radar_id_expansion.h"
#include "modules/perception/obstacle/radar/modest/modest_radar_detector.h"
#include "modules/perception/onboard/subnode.h"
#include "modules/perception/proto/perception_obstacle.pb.h"

namespace apollo {
namespace perception {

class RadarProcessSubnode : public Subnode {
 public:
  RadarProcessSubnode() = default;
  ~RadarProcessSubnode() = default;

  apollo::common::Status ProcEvents() override {
    return apollo::common::Status::OK();
  }

 private:
  typedef std::pair<double, apollo::localization::LocalizationEstimate>
      LocalizationPair;
  bool InitInternal() override;

  void OnRadar(const ContiRadar &radar_obs);

  void OnLocalization(
      const apollo::localization::LocalizationEstimate &localization);

  void RegistAllAlgorithm();

  bool InitFrameDependence();

  bool InitAlgorithmPlugin();

  void PublishDataAndEvent(double timestamp,
                           const SharedDataPtr<SensorObjects> &data);

  bool GetCarLinearSpeed(double timestamp, Eigen::Vector3f *car_linear_speed);

  bool inited_ = false;
  SeqId seq_num_ = 0;
  common::ErrorCode error_code_ = common::OK;
  RadarObjectData *radar_data_ = nullptr;
  std::string device_id_;
  Eigen::Matrix4d radar_extrinsic_;
  Eigen::Matrix4d short_camera_extrinsic_;

  boost::circular_buffer<LocalizationPair> localization_buffer_;
  ContiRadarIDExpansion conti_id_expansion_;
  std::unique_ptr<BaseRadarDetector> radar_detector_;
  HDMapInput *hdmap_input_ = nullptr;
  // here we use HdmapROIFilter
  std::unique_ptr<HdmapROIFilter> roi_filter_;
  Mutex mutex_;
};

REGISTER_SUBNODE(RadarProcessSubnode);

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_ONBOARD_SUBNODE_H_
