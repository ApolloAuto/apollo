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

#include "cyber/component/component.h"
#include "modules/drivers/proto/sensor_image.pb.h"
#include "modules/map/proto/map_geometry.pb.h"
#include "modules/map/proto/map_signal.pb.h"
#include "modules/perception/camera/app/traffic_light_camera_perception.h"
#include "modules/perception/camera/common/util.h"
#include "modules/perception/camera/lib/traffic_light/preprocessor/tl_preprocessor.h"
#include "modules/perception/map/hdmap/hdmap_input.h"
#include "modules/perception/onboard/proto/trafficlights_perception_component.pb.h"
#include "modules/perception/onboard/transform_wrapper/transform_wrapper.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/perception/proto/traffic_light_detection.pb.h"
#include "modules/transform/buffer.h"

#include "modules/v2x/common/v2x_proxy_gflags.h"
#include "modules/v2x/proto/v2x_traffic_light.pb.h"

namespace apollo {
namespace perception {
namespace onboard {

class TrafficLightsPerceptionComponent : public apollo::cyber::Component<> {
 public:
  TrafficLightsPerceptionComponent() = default;
  ~TrafficLightsPerceptionComponent() = default;

  TrafficLightsPerceptionComponent(const TrafficLightsPerceptionComponent&) =
      delete;
  TrafficLightsPerceptionComponent& operator=(
      const TrafficLightsPerceptionComponent&) = delete;

  bool Init() override;

 private:
  int InitConfig();
  int InitAlgorithmPlugin();
  int InitCameraListeners();
  int InitCameraFrame();

  int InitV2XListener();

  void OnReceiveImage(const std::shared_ptr<apollo::drivers::Image> image,
                      const std::string& camera_name);

  void OnReceiveV2XMsg(
      const std::shared_ptr<apollo::v2x::IntersectionTrafficLightData> v2x_msg);

  bool QueryPoseAndSignals(const double ts, camera::CarPose* pose,
                           std::vector<apollo::hdmap::Signal>* signals);

  bool VerifyLightsProjection(const double& ts,
                              const camera::TLPreprocessorOption& option,
                              const std::string& camera_name,
                              camera::CameraFrame* image_lights);

  bool UpdateCameraSelection(double timestamp,
                             const camera::TLPreprocessorOption& option,
                             camera::CameraFrame* frame);

  bool CheckCameraImageStatus(double timestamp, double interval,
                              const std::string& camera_name);

  bool GetCarPose(const double timestamp, camera::CarPose* pose);

  bool GetPoseFromTF(const double timestamp, const std::string& frame_id,
                     const std::string& child_frame_id,
                     Eigen::Matrix4d* pose_matrix);

  double stopline_distance(const Eigen::Matrix4d& cam_pose);
  bool TransformOutputMessage(
      camera::CameraFrame* frame, const std::string& camera_name,
      std::shared_ptr<apollo::perception::TrafficLightDetection>* out_msg);

  bool TransformDebugMessage(
      const camera::CameraFrame* frame,
      std::shared_ptr<apollo::perception::TrafficLightDetection>* out_msg);
  void SendSimulationMsg();
  void GenerateTrafficLights(
      const std::vector<apollo::hdmap::Signal>& signals,
      std::vector<base::TrafficLightPtr>* traffic_lights);
  void TransRect2Box(const base::RectI& rect,
                     apollo::perception::TrafficLightBox* box);

 private:
  void Visualize(const camera::CameraFrame& frame,
                 const std::vector<base::TrafficLightPtr>& lights) const;
  void SyncV2XTrafficLights(camera::CameraFrame* frame);

 private:
  std::mutex mutex_;

  std::shared_ptr<camera::TLPreprocessor> preprocessor_;
  apollo::perception::map::HDMapInput* hd_map_ = nullptr;

  camera::TrafficLightPreprocessorInitOptions preprocessor_init_options_;

  std::string tf2_frame_id_;
  std::string tf2_child_frame_id_;
  double tf2_timeout_second_ = 0.01;

  Buffer* tf2_buffer_ = Buffer::Instance();

  std::vector<std::string> camera_names_;
  std::vector<std::string> input_camera_channel_names_;
  // camera_name -> TransformWrapper
  std::map<std::string, std::shared_ptr<TransformWrapper>>
      camera2world_trans_wrapper_map_;
  // camera_name -> image_border_size
  std::map<std::string, int> image_border_sizes_;

  std::vector<std::shared_ptr<cyber::Node>> camera_listener_nodes_;

  double last_sub_tf_ts_ = 0.0;

  std::map<std::string, double> last_sub_camera_image_ts_;

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
  int image_channel_num_ = 3;
  int image_data_size_ = -1;
  int frame_index_ = 0;
  int default_image_border_size_ = 100;

  // options for DataProvider
  bool enable_undistortion_ = false;
  camera::DataProvider::InitOptions data_provider_init_options_;

  // pre-allocated-mem data_provider; camera_id -> data_provider
  std::map<std::string, std::shared_ptr<camera::DataProvider>>
      data_providers_map_;

  // image
  std::shared_ptr<camera::CameraFrame> frame_;

  // proc
  camera::CameraPerceptionInitOptions camera_perception_init_options_;
  camera::CameraPerceptionOptions camera_perception_options_;
  std::unique_ptr<camera::TrafficLightCameraPerception> traffic_light_pipeline_;
  ::google::protobuf::RepeatedPtrField<apollo::hdmap::Curve> stoplines_;

  // msg channel name
  std::string simulation_channel_name_;
  std::string traffic_light_output_channel_name_;

  std::shared_ptr<
      apollo::cyber::Writer<apollo::perception::TrafficLightDetection>>
      writer_;

  // traffic lights
  apollo::perception::base::TLColor detected_trafficlight_color_;
  double cnt_r_;
  double cnt_g_;
  double cnt_y_;
  double cnt_u_;

  // v2x
  std::string v2x_trafficlights_input_channel_name_;
  double v2x_sync_interval_seconds_ = 0.1;
  int max_v2x_msg_buff_size_ = 50;
  boost::circular_buffer<apollo::v2x::IntersectionTrafficLightData>
      v2x_msg_buffer_;
};

CYBER_REGISTER_COMPONENT(TrafficLightsPerceptionComponent);

}  // namespace onboard
}  // namespace perception
}  // namespace apollo
