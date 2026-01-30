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

#include "cyber/time/clock.h"
#include "cyber/profiler/profiler.h"
#include "modules/perception/common/camera/common/trafficlight_frame.h"
#include "modules/perception/traffic_light_recognition/interface/base_traffic_light_recognitor.h"
#include "modules/perception/traffic_light_recognition/proto/traffic_light_recognition_component.pb.h"

namespace apollo {
namespace perception {
namespace trafficlight {

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
    AINFO << "Successfully init traffic light recognition component.";
    return true;
}

bool TrafficLightRecognComponent::Proc(const std::shared_ptr<TrafficDetectMessage>& message) {
    PERF_FUNCTION()
    auto time_imags = std::to_string(message->timestamp_);
    AINFO << "Enter recognition component, message timestamp: " << time_imags;

    bool status = InternalProc(message);
    if (status) {
        writer_->Write(message);
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
    AINFO << "tl_recognitor_name: " << tl_recognitor_name_ << " config_path: " << config_path_
          << " config_file: " << config_file_ << " gpu_id: " << gpu_id_;

    writer_ = node_->CreateWriter<TrafficDetectMessage>(traffic_light_param.recognition_output_channel_name());

    return cyber::SUCC;
}

bool TrafficLightRecognComponent::InitAlgorithmPlugin() {
    trafficlight::BaseTrafficLightRecognitor* recognitor
            = trafficlight::BaseTrafficLightRecognitorRegisterer::GetInstanceByName(tl_recognitor_name_);
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

bool TrafficLightRecognComponent::InternalProc(const std::shared_ptr<TrafficDetectMessage const>& message) {
    PERF_BLOCK("traffic_light_recognition")
    bool status = recognitor_->Detect(message->traffic_light_frame_.get());
    PERF_BLOCK_END

    return true;
}

}  // namespace trafficlight
}  // namespace perception
}  // namespace apollo
