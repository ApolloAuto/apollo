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
#include "modules/perception/camera_detection_bev/proto/camera_detection_bev.pb.h"

#include "cyber/cyber.h"
#include "modules/perception/camera_detection_bev/camera_frame.h"
#include "modules/perception/camera_detection_bev/interface/base_obstacle_detector.h"
#include "modules/perception/common/onboard/transform_wrapper/transform_wrapper.h"

namespace apollo {
namespace perception {
namespace camera {

class CameraDetectionBevComponent final
    : public cyber::Component<drivers::Image> {
 public:
  CameraDetectionBevComponent() : timestamp_offset_(0) {}
  /**
   * @brief Init for camera detection 2d compoment
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
  bool Proc(const std::shared_ptr<drivers::Image>& msg) override;

 private:
  bool InitTransformWrapper(const CameraDetectionBEV& detection_param);
  bool InitCameraFrame(const CameraDetectionBEV& detection_param);
  bool InitListeners(const CameraDetectionBEV& detection_param);
  bool InitDetector(const CameraDetectionBEV& detection_param);

  void CameraToWorldCoor(const Eigen::Affine3d& camera2world,
                         std::vector<base::ObjectPtr>* objs);

 private:
  int image_height_;
  int image_width_;
  double timestamp_offset_;

  std::shared_ptr<CameraFrame> frame_ptr_;
  std::shared_ptr<BaseObstacleDetector> detector_;
  std::shared_ptr<onboard::TransformWrapper> trans_wrapper_;

  std::vector<std::shared_ptr<cyber::Reader<drivers::Image>>> readers_;
  std::shared_ptr<cyber::Writer<CameraFrame>> writer_;
};

CYBER_REGISTER_COMPONENT(CameraDetectionBevComponent);

}  // namespace camera
}  // namespace perception
}  // namespace apollo
