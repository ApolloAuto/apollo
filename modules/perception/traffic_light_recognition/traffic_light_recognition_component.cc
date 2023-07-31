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
#include "modules/perception/traffic_light_recognition/traffic_light_recognition_component.h"

#include "modules/perception/traffic_light_recognition/proto/traffic_light_recognition_component.pb.h"

#include "cyber/time/clock.h"
#include "modules/perception/common/camera/common/trafficlight_frame.h"
#include "modules/perception/traffic_light_recognition/interface/base_traffic_light_recognitor.h"

namespace apollo {
namespace perception {
namespace onboard {

bool TrafficLightRecognComponent::Init() {
  // init component config
  if (InitConfig() != cyber::SUCC) {
    AERROR << "TrafficLightRecognComponent InitConfig failed.";
    return false;
  }
  // init algorithm plugin
  if (InitAlgorithmPlugin() != cyber::SUCC) {
    AERROR << "TrafficLightRecognComponent InitAlgorithmPlugin failed.";
    return false;
  }

  return true;
}

bool TrafficLightRecognComponent::Proc(
    const std::shared_ptr<TrafficDetectMessage>& message) {
  auto time_imags = std::to_string(message->timestamp_);
  AINFO << "Enter recognition component, message timestamp: " << time_imags;

  std::shared_ptr<TrafficDetectMessage> out_message(new (std::nothrow)
                                                        TrafficDetectMessage);
  bool status = InternalProc(message, out_message);
  if (status) {
    writer_->Write(out_message);
    AINFO << "Send trafficlight recognition output message.";
  }
  return status;
}

int TrafficLightRecognComponent::InitConfig() {
  apollo::perception::trafficlight::RecognitionParam traffic_light_param;
  if (!GetProtoConfig(&traffic_light_param)) {
    AINFO << "load trafficlights recognition component proto param failed";
    return cyber::FAIL;
  }

  PluginParam plugin_param = traffic_light_param.plugin_param();
  tl_recognitor_name_ = plugin_param.name();
  config_path_ = plugin_param.config_path();
  config_file_ = plugin_param.config_file();

  gpu_id_ = traffic_light_param.gpu_id();
  AINFO << "tl_recognitor_name: " << tl_recognitor_name_
        << " config_path: " << config_path_ << " config_file: " << config_file_
        << " gpu_id: " << gpu_id_;

  writer_ = node_->CreateWriter<TrafficDetectMessage>(
      traffic_light_param.recognition_output_channel_name());

  return cyber::SUCC;
}

bool TrafficLightRecognComponent::InitAlgorithmPlugin() {
  trafficlight::BaseTrafficLightRecognitor* recognitor =
      trafficlight::BaseTrafficLightRecognitorRegisterer::GetInstanceByName(
          tl_recognitor_name_);
  CHECK_NOTNULL(recognitor);
  recognitor_.reset(recognitor);
  ACHECK(recognitor_ != nullptr);
  recognitor_init_options_.gpu_id = gpu_id_;
  recognitor_init_options_.config_path = config_path_;
  recognitor_init_options_.config_file = config_file_;

  if (!recognitor_->Init(recognitor_init_options_)) {
    AERROR << "Trafficlight recognitor init failed";
    return false;
  }

  return cyber::SUCC;
}

bool TrafficLightRecognComponent::InternalProc(
    const std::shared_ptr<TrafficDetectMessage const>& in_message,
    std::shared_ptr<TrafficDetectMessage> out_message) {
  camera::TrafficLightFrame traffic_light_frame;
  traffic_light_frame.timestamp = in_message->timestamp_;
  traffic_light_frame.data_provider =
      in_message->traffic_light_frame_->data_provider;
  traffic_light_frame.traffic_lights =
      in_message->traffic_light_frame_->traffic_lights;

  bool status = recognitor_->Detect(&traffic_light_frame);
  if (!status) {
    out_message->error_code_ =
        apollo::common::ErrorCode::PERCEPTION_ERROR_PROCESS;
    AERROR << "Trafficlight recognition process error!";
    return false;
  }

  out_message->timestamp_ = in_message->timestamp_;
  out_message->stoplines_ = in_message->stoplines_;
  // fill car pose info
  auto& carpose = out_message->carpose_;
  carpose.reset(new camera::CarPose);
  carpose = in_message->carpose_;

  auto& frame = out_message->traffic_light_frame_;
  frame.reset(new camera::TrafficLightFrame);
  frame->timestamp = in_message->timestamp_;
  frame->data_provider = traffic_light_frame.data_provider;
  frame->traffic_lights = traffic_light_frame.traffic_lights;

  return true;
}

}  // namespace onboard
}  // namespace perception
}  // namespace apollo
