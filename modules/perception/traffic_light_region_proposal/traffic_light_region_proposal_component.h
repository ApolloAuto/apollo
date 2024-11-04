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
#include <string>
#include <vector>

#include "modules/common_msgs/map_msgs/map_geometry.pb.h"
#include "modules/common_msgs/map_msgs/map_signal.pb.h"
#include "modules/common_msgs/perception_msgs/perception_obstacle.pb.h"
#include "modules/common_msgs/perception_msgs/traffic_light_detection.pb.h"
#include "modules/common_msgs/sensor_msgs/sensor_image.pb.h"
#include "modules/common_msgs/transform_msgs/transform.pb.h"
#include "modules/perception/traffic_light_region_proposal/proto/trafficlights_proposal_component.pb.h"

#include "cyber/component/component.h"
#include "modules/perception/common/camera/common/util.h"
#include "modules/perception/common/hdmap/hdmap_input.h"
#include "modules/perception/common/onboard/inner_component_messages/traffic_inner_component_messages.h"
#include "modules/perception/common/onboard/transform_wrapper/transform_wrapper.h"
#include "modules/perception/traffic_light_region_proposal/preprocessor/tl_preprocessor.h"
#include "modules/transform/buffer.h"

namespace apollo {
namespace perception {
namespace trafficlight {

using apollo::perception::onboard::TrafficDetectMessage;
using apollo::perception::onboard::TransformWrapper;
using apollo::transform::Buffer;

class TrafficLightsPerceptionComponent : public apollo::cyber::Component<> {
 public:
  /**
   * @brief Construct a new traffic lights perception component object.
   *
   */
  TrafficLightsPerceptionComponent() = default;
  /**
   * @brief Destroy the traffic lights perception component object.
   *
   */
  ~TrafficLightsPerceptionComponent() = default;

  TrafficLightsPerceptionComponent(const TrafficLightsPerceptionComponent&) =
      delete;
  TrafficLightsPerceptionComponent& operator=(
      const TrafficLightsPerceptionComponent&) = delete;
  /**
   * @brief Initialize configuration files, algorithm plug-ins,
            callback functions and create listening channels.
   *
   * @return true
   * @return false
   */
  bool Init() override;

 private:
  int InitConfig();
  int InitAlgorithmPlugin();
  int InitCameraListeners();
  int InitCameraFrame();

  void OnReceiveImage(const std::shared_ptr<apollo::drivers::Image> image,
                      const std::string& camera_name);

  bool QueryPoseAndSignals(const double ts, camera::CarPose* pose,
                           std::vector<apollo::hdmap::Signal>* signals);

  bool VerifyLightsProjection(
      const double& ts, const trafficlight::TLPreprocessorOption& option,
      const std::string& camera_name,
      std::shared_ptr<camera::TrafficLightFrame> image_lights,
      std::shared_ptr<TrafficDetectMessage> msg);

  bool UpdateCameraSelection(double timestamp,
                             const trafficlight::TLPreprocessorOption& option,
                             std::shared_ptr<camera::TrafficLightFrame> frame);

  bool CheckCameraImageStatus(double timestamp, double interval,
                              const std::string& camera_name);

  bool GetCarPose(const double timestamp, camera::CarPose* pose);

  bool GetPoseFromTF(const double timestamp, const std::string& frame_id,
                     const std::string& child_frame_id,
                     Eigen::Matrix4d* pose_matrix);

  void GenerateTrafficLights(
      const std::vector<apollo::hdmap::Signal>& signals,
      std::vector<base::TrafficLightPtr>* traffic_lights);

 private:
  std::mutex mutex_;

  std::shared_ptr<trafficlight::BaseTLPreprocessor> preprocessor_;
  apollo::perception::map::HDMapInput* hd_map_ = nullptr;

  trafficlight::TrafficLightPreprocessorInitOptions preprocessor_init_options_;
  std::string tl_preprocessor_name_;

  std::string tf2_frame_id_;
  std::string tf2_child_frame_id_;
  std::string proposal_output_channel_name_;
  int gpu_id_;
  double tf2_timeout_second_ = 0.01;

  Buffer* tf2_buffer_ = Buffer::Instance();

  std::vector<std::string> camera_names_;
  std::vector<std::string> input_camera_channel_names_;
  // camera_name -> TransformWrapper
  std::map<std::string, std::shared_ptr<TransformWrapper>>
      camera2world_trans_wrapper_map_;
  // camera_name -> image_border_size
  std::map<std::string, int> image_border_sizes_;
  std::map<std::string, double> last_sub_camera_image_ts_;

  // pre-allocated-mem data_provider; camera_id -> data_provider
  std::map<std::string, std::shared_ptr<camera::DataProvider>>
      data_providers_map_;

  double query_tf_interval_seconds_ = 0.0;
  double image_timestamp_offset_ = 0.0;
  int max_process_image_fps_ = 10;  // max frames to be processed per second
  double proc_interval_seconds_ = 0.0;
  double check_image_status_interval_thresh_ = 1.0;

  double last_query_tf_ts_ = 0.0;
  double last_proc_image_ts_ = 0.0;
  double image_sys_ts_diff_threshold_ = 0.5;

  // for querying hdmap signals
  double last_signals_ts_ = -1.0;
  double valid_hdmap_interval_ = 1.5;
  double forward_distance_to_query_signals = 150.0;
  std::vector<apollo::hdmap::Signal> last_signals_;

  // image info.
  int image_width_ = 1920;
  int image_height_ = 1080;
  int default_image_border_size_ = 100;

  // options for DataProvider
  bool enable_undistortion_ = false;
  camera::DataProvider::InitOptions data_provider_init_options_;

  // proc
  ::google::protobuf::RepeatedPtrField<apollo::hdmap::Curve> stoplines_;

  std::shared_ptr<apollo::cyber::Writer<TrafficDetectMessage>>
      traffic_detect_writer_;
};

CYBER_REGISTER_COMPONENT(TrafficLightsPerceptionComponent);

}  // namespace trafficlight
}  // namespace perception
}  // namespace apollo
