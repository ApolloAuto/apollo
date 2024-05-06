/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/StdVector"

#include "modules/common_msgs/perception_msgs/perception_lane.pb.h"
#include "modules/common_msgs/sensor_msgs/sensor_image.pb.h"
#include "modules/perception/common/proto/motion_service.pb.h"
#include "modules/perception/lane_detection/proto/lane_perception_component.pb.h"
#include "modules/perception/lane_detection/proto/perception.pb.h"

#include "cyber/component/component.h"
#include "modules/common/util/eigen_defs.h"
#include "modules/common/util/util.h"
#include "modules/perception/common/base/object.h"
#include "modules/perception/common/base/object_types.h"
#include "modules/perception/common/base/point.h"
#include "modules/perception/common/onboard/inner_component_messages/inner_component_messages.h"
#include "modules/perception/common/onboard/transform_wrapper/transform_wrapper.h"
#include "modules/perception/common/util.h"
#include "modules/perception/lane_detection/app/lane_camera_perception.h"
#include "modules/perception/lane_detection/interface/base_camera_perception.h"
#include "modules/perception/tools/offline/visualizer.h"

namespace apollo {
namespace perception {
namespace onboard {


class LaneDetectionComponent;
typedef FunctionInfo<LaneDetectionComponent> FunInfoType;
class LaneDetectionComponent : public apollo::cyber::Component<> {
 public:
  template <class EigenType>
  using EigenVector = apollo::common::EigenVector<EigenType>;

  template <typename T, class EigenType>
  using EigenMap = apollo::common::EigenMap<T, EigenType>;

  using MotionServiceMsgType =
      std::shared_ptr<apollo::perception::MotionService>;

 public:
  LaneDetectionComponent() : seq_num_(0) {}
  ~LaneDetectionComponent();

  LaneDetectionComponent(const LaneDetectionComponent&) = delete;
  LaneDetectionComponent& operator=(const LaneDetectionComponent&) = delete;

  bool Init() override;

  template <typename T>
  friend class FunctionInfo;

 private:
  void OnReceiveImage(const std::shared_ptr<apollo::drivers::Image>& in_message,
                      const std::string& camera_name);
  void OnMotionService(const MotionServiceMsgType& in_message);
  int InitConfig();
  int InitSensorInfo();
  int InitAlgorithmPlugin();
  int InitCameraFrames();
  int InitProjectMatrix();
  int InitMotionService();
  int InitCameraListeners();
  void SetCameraHeightAndPitch();

  int InternalProc(
      const std::shared_ptr<apollo::drivers::Image const>& in_message,
      const std::string& camera_name, apollo::common::ErrorCode* error_code,
      SensorFrameMessage* prefused_message,
      apollo::perception::PerceptionLanes* out_message);

  int ConvertLaneToCameraLaneline(
      const base::LaneLine& lane_line,
      apollo::perception::camera::CameraLaneLine* camera_laneline);

  int MakeProtobufMsg(double msg_timestamp, const std::string& camera_name,
                      const camera::CameraFrame& camera_frame,
                      apollo::perception::PerceptionLanes* lanes_msg);

 private:
  std::mutex mutex_;
  uint32_t seq_num_;

  std::vector<std::shared_ptr<cyber::Node>> camera_listener_nodes_;

  std::vector<std::string> camera_names_;  // camera sensor names
  std::vector<std::string> input_camera_channel_names_;

  // camera name -> SensorInfo
  std::map<std::string, base::SensorInfo> sensor_info_map_;

  // camera_height
  std::map<std::string, float> camera_height_map_;

  // camera_pitch_angle_diff
  std::map<std::string, float> name_camera_pitch_angle_diff_map_;

  // TF stuff
  std::map<std::string, std::string> tf_camera_frame_id_map_;
  std::map<std::string, std::shared_ptr<TransformWrapper>>
      camera2world_trans_wrapper_map_;

  // pre-allocaated-mem data_provider;
  std::map<std::string, std::shared_ptr<camera::DataProvider>>
      data_providers_map_;

  // map for store params
  EigenMap<std::string, Eigen::Matrix4d> extrinsic_map_;
  EigenMap<std::string, Eigen::Matrix3f> intrinsic_map_;
  Eigen::Matrix3d homography_image2ground_;

  // camera lane pipeline
  camera::CameraPerceptionInitOptions camera_perception_init_options_;
  std::unique_ptr<camera::LaneCameraPerception> camera_lane_pipeline_;

  // fixed size camera frames
  int frame_capacity_ = 20;
  int frame_id_ = 0;
  EigenVector<camera::CameraFrame> camera_frames_;

  // image info.
  int image_width_ = 1920;
  int image_height_ = 1080;
  int image_channel_num_ = 3;
  int image_data_size_ = -1;

  // default camera pitch angle & height
  float default_camera_pitch_ = 0.f;
  float default_camera_height_ = 1.6f;

  // options for DataProvider
  bool enable_undistortion_ = false;

  double timestamp_offset_ = 0.0;

  bool enable_visualization_ = false;
  std::string visual_debug_folder_;
  std::string visual_camera_;

  std::string output_lanes_channel_name_;

  Eigen::Matrix3d project_matrix_;
  double pitch_diff_ = 0.0;

  double last_timestamp_ = 0.0;
  double ts_diff_ = 1.0;

  std::shared_ptr<apollo::cyber::Writer<apollo::perception::PerceptionLanes>>
      writer_;

  base::MotionBufferPtr mot_buffer_;
  const int motion_buffer_size_ = 100;

  camera::Visualizer visualize_;
  bool write_visual_img_;
  static FunInfoType init_func_arry_[];
};

CYBER_REGISTER_COMPONENT(LaneDetectionComponent);

}  // namespace onboard
}  // namespace perception
}  // namespace apollo
