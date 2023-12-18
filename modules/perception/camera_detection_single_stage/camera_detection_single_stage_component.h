/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#pragma once

#include <memory>
#include <vector>

#include "modules/common_msgs/sensor_msgs/sensor_image.pb.h"
#include "modules/perception/camera_detection_single_stage/proto/camera_detection_single_stage.pb.h"

#include "cyber/cyber.h"
#include "modules/perception/common/interface/base_obstacle_detector.h"
#include "modules/perception/common/onboard/inner_component_messages/camera_detection_component_messages.h"
#include "modules/perception/common/onboard/transform_wrapper/transform_wrapper.h"

namespace apollo {
namespace perception {
namespace camera {

class CameraDetectionSingleStageComponent final
    : public cyber::Component<apollo::drivers::Image> {
 public:
  CameraDetectionSingleStageComponent() : timestamp_offset_(0) {}
  ~CameraDetectionSingleStageComponent() = default;
  /**
   * @brief Init for amera detection 3d compoment
   *
   * @return true
   * @return false
   */
  bool Init() override;
  /**
   * @brief Process of camera detection 2d compoment
   *
   * @param msg image msg
   * @return true
   * @return false
   */
  bool Proc(const std::shared_ptr<apollo::drivers::Image>& msg) override;

 private:
  bool InitObstacleDetector(const CameraDetectionSingleStage& detection_param);
  bool InitCameraFrame(const CameraDetectionSingleStage& detection_param);
  bool InitTransformWrapper(const CameraDetectionSingleStage& detection_param);
  void CameraToWorldCoor(const Eigen::Affine3d& camera2world,
                         std::vector<base::ObjectPtr>* objs);

  bool InternalProc(const std::shared_ptr<apollo::drivers::Image>& msg,
                    const std::shared_ptr<onboard::CameraFrame>& out_message);

 private:
  int image_height_;
  int image_width_;
  int frame_id_ = 0;
  double timestamp_offset_;
  Eigen::Matrix3f camera_k_matrix_ = Eigen::Matrix3f::Identity();

  std::shared_ptr<BaseObstacleDetector> detector_;
  std::shared_ptr<onboard::TransformWrapper> trans_wrapper_;
  std::shared_ptr<camera::DataProvider> data_provider_;

  std::shared_ptr<cyber::Writer<onboard::CameraFrame>> writer_;
};

CYBER_REGISTER_COMPONENT(CameraDetectionSingleStageComponent);

}  // namespace camera
}  // namespace perception
}  // namespace apollo
