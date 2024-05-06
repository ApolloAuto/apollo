/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "modules/common_msgs/perception_msgs/traffic_light_detection.pb.h"
#include "modules/common_msgs/v2x_msgs/v2x_traffic_light.pb.h"

#include "cyber/component/component.h"
#include "modules/perception/common/camera/common/pose.h"
#include "modules/perception/common/camera/common/trafficlight_frame.h"
#include "modules/perception/common/onboard/inner_component_messages/traffic_inner_component_messages.h"
#include "modules/perception/traffic_light_tracking/interface/base_traffic_light_tracker.h"

namespace apollo {
namespace perception {
namespace trafficlight {

using apollo::perception::onboard::TrafficDetectMessage;

class TrafficLightTrackComponent
    : public cyber::Component<TrafficDetectMessage> {
 public:
  TrafficLightTrackComponent() = default;
  ~TrafficLightTrackComponent() = default;
  bool Init() override;
  bool Proc(const std::shared_ptr<TrafficDetectMessage>& message) override;

 private:
  int InitConfig();
  int InitV2XListener();
  bool InitAlgorithmPlugin();

  bool InternalProc(
      const std::shared_ptr<TrafficDetectMessage const>& message);

  void OnReceiveV2XMsg(
      const std::shared_ptr<apollo::v2x::IntersectionTrafficLightData> v2x_msg);

  void SyncV2XTrafficLights(camera::TrafficLightFrame* frame);

  double stopline_distance(const Eigen::Matrix4d& cam_pose);

  void TransRect2Box(const base::RectI& rect,
                     apollo::perception::TrafficLightBox* box);

  void Visualize(const camera::TrafficLightFrame& frame,
                 const std::vector<base::TrafficLightPtr>& lights) const;

  bool TransformOutputMessage(
      camera::TrafficLightFrame* frame, const std::string& camera_name,
      std::shared_ptr<apollo::perception::TrafficLightDetection>* out_msg,
      const std::shared_ptr<TrafficDetectMessage const>& message);

  bool TransformDebugMessage(
      const camera::TrafficLightFrame* frame,
      std::shared_ptr<apollo::perception::TrafficLightDetection>* out_msg,
      const std::shared_ptr<TrafficDetectMessage const>& message);

 private:
  std::mutex mutex_;
  // tracker
  trafficlight::TrafficLightTrackerInitOptions tracker_init_options_;
  std::string tl_tracker_name_;
  std::string config_path_;
  std::string config_file_;
  std::string traffic_light_output_channel_name_;

  // traffic lights
  apollo::perception::base::TLColor detected_trafficlight_color_;
  double cnt_r_;
  double cnt_g_;
  double cnt_y_;
  double cnt_u_;

  // image info.
  int image_width_ = 1920;
  int image_height_ = 1080;

  // v2x
  int max_v2x_msg_buff_size_ = 50;
  std::string v2x_trafficlights_input_channel_name_;
  double v2x_sync_interval_seconds_ = 0.1;
  boost::circular_buffer<apollo::v2x::IntersectionTrafficLightData>
      v2x_msg_buffer_;

  // proc
  ::google::protobuf::RepeatedPtrField<apollo::hdmap::Curve> stoplines_;
  std::shared_ptr<trafficlight::BaseTrafficLightTracker> tracker_;
  std::shared_ptr<
      apollo::cyber::Writer<apollo::perception::TrafficLightDetection>>
      writer_;
};

CYBER_REGISTER_COMPONENT(TrafficLightTrackComponent);

}  // namespace trafficlight
}  // namespace perception
}  // namespace apollo
