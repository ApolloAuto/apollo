/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
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
#include <string>
#include <vector>

#include "yaml-cpp/yaml.h"

#include "modules/common_msgs/perception_msgs/perception_obstacle.pb.h"
#include "modules/common_msgs/sensor_msgs/sensor_image.pb.h"
#include "modules/perception/camera_detection_occupancy/proto/camera_detection_occupancy.pb.h"

#include "cyber/cyber.h"
#include "modules/perception/camera_detection_occupancy/camera_frame.h"
#include "modules/perception/camera_detection_occupancy/interface/base_obstacle_detector.h"
#include "modules/perception/camera_detection_occupancy/interface/base_tracker.h"
#include "modules/perception/common/onboard/transform_wrapper/transform_wrapper.h"

namespace apollo {
namespace perception {
namespace camera {

class CameraDetectionOccComponent final : public cyber::Component<> {
 public:
  using PerceptionObstacle = apollo::perception::PerceptionObstacle;
  using PerceptionObstacles = apollo::perception::PerceptionObstacles;

 public:
  CameraDetectionOccComponent() : timestamp_offset_(0) {}
  /**
   * @brief Init for camera detection 2d compoment
   *
   * @return true
   * @return false
   */
  bool Init() override;

 private:
  bool InitTransformWrapper(const CameraDetectionBEV& detection_param);
  bool InitCameraFrame(const CameraDetectionBEV& detection_param);
  bool InitListeners(const CameraDetectionBEV& detection_param);
  bool InitDetector(const CameraDetectionBEV& detection_param);

  bool OnReceiveImage(const std::shared_ptr<drivers::Image>& msg);

  void CameraToWorldCoor(const Eigen::Affine3d& camera2world,
                         std::vector<base::ObjectPtr>* objs);
  int ConvertObjectToPb(const base::ObjectPtr& object_ptr,
                        PerceptionObstacle* pb_msg);
  int MakeProtobufMsg(double msg_timestamp, int seq_num,
                      const std::vector<base::ObjectPtr>& objects,
                      PerceptionObstacles* obstacles);
  bool LoadCameraExtrinsic(const std::string& file_path,
                           Eigen::Affine3d* camera_extrinsic);
  bool LoadCameraExtrinsicNus(
                const std::string &camera_extrinsic_file_path,
                const std::string &lidar_extrinsic_file_path,
                Eigen::Affine3d *camera2lidar_rt);
  bool ConvertToBaseFrame(const CameraFrame* frame, base::Frame* base_frame);

 private:
  int image_height_;
  int image_width_;
  uint32_t seq_num_ = 0;
  double timestamp_offset_ = 0;
  ObstacleDetectorInitOptions init_options;
  TrackerInitOptions tracker_init_options;

  std::shared_ptr<CameraFrame> frame_ptr_;
  std::shared_ptr<base::Frame> base_frame_ptr_;
  std::shared_ptr<BaseObstacleDetector> detector_;
  std::shared_ptr<BaseTracker> tracker_;
  // ObjectBuilder builder_;
  std::shared_ptr<onboard::TransformWrapper> trans_wrapper_;

  std::vector<std::shared_ptr<cyber::Reader<drivers::Image>>> readers_;
  std::shared_ptr<cyber::Writer<PerceptionObstacles>> writer_;
};

CYBER_REGISTER_COMPONENT(CameraDetectionOccComponent);

}  // namespace camera
}  // namespace perception
}  // namespace apollo
