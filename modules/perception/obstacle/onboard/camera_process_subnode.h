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

#ifndef MODULES_PERCEPTION_OBSTACLE_ONBORAD_LIDAR_PROCESS_SUBNODE_H_
#define MODULES_PERCEPTION_OBSTACLE_ONBORAD_LIDAR_PROCESS_SUBNODE_H_

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "sensor_msgs/Image.h"

#include "modules/perception/proto/perception_obstacle.pb.h"

#include "modules/common/log.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/perception/obstacle/base/object.h"
#include "modules/perception/obstacle/camera/interface/base_camera_converter.h"
#include "modules/perception/obstacle/camera/interface/base_camera_detector.h"
#include "modules/perception/obstacle/camera/interface/base_camera_filter.h"
#include "modules/perception/obstacle/camera/interface/base_camera_tracker.h"
#include "modules/perception/obstacle/camera/interface/base_camera_transformer.h"
#include "modules/perception/obstacle/camera/interface/base_lane_post_processor.h"
#include "modules/perception/obstacle/onboard/object_shared_data.h"
#include "modules/perception/onboard/subnode.h"

#include <cv_bridge/cv_bridge.h>
#include <yaml-cpp/yaml.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "lib/base/macros.h"
#include "lib/config_manager/config_manager.h"
#include "onboard/event_manager.h"
#include "onboard/shared_data_manager.h"
#include "onboard/types.h"

namespace apollo {
namespace perception {

class CameraProcessSubnode : public Subnode {
 public:
  CameraProcessSubnode() = default;
  ~CameraProcessSubnode() = default;

  apollo::common::Status ProcEvents() override {
    return apollo::common::Status::OK();
  }

 private:
  bool InitInternal() override;

  bool InitCalibration();

  bool InitModules();

  void ImgCallback(const sensor_msgs::Image::ConstPtr& message);

  bool MessageToMat(const sensor_msgs::Image::ConstPtr& message, cv::Mat* mat);

  void VisualObjToSensorObj(
      const std::vector<VisualObjectPtr> &objects,
      onboard::SharedDataPtr<SensorObjects>* sensor_objects);

  void PublishDataAndEvent(
      const float &timestamp,
      const onboard::SharedDataPtr<SensorObjects>& sensor_objects,
      const onboard::SharedDataPtr<CameraItem>& camera_item);

  SeqId seq_num_ = 0;
  std::string device_id_;

  // Shared Data
  std::shared_ptr<CameraObjectData> cam_obj_data_;
  std::shared_ptr<CameraSharedData> cam_shared_data_;

  // Calibration
  Eigen::Matrix4d camera_to_car_;
  Eigen::Matrix<double, 3, 4> intrinsics_;
  adu::perception::config_manager::CameraUndistortionPtr undistortion_handler_;

  // Modules
  std::unique_ptr<BaseCameraDetector> detector_;
  std::unique_ptr<BaseCameraConverter> converter_;
  std::unique_ptr<BaseCameraTracker> tracker_;
  std::unique_ptr<BaseCameraTransformer> transformer_;
  std::unique_ptr<BaseCameraFilter> filter_;
};

REGISTER_SUBNODE(CameraProcessSubnode);

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_ONBORAD_LIDAR_PROCESS_SUBNODE_H_
