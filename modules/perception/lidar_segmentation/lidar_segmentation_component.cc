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

#include "modules/perception/lidar_segmentation/lidar_segmentation_component.h"

namespace apollo {
namespace perception {
namespace lidar {

using apollo::cyber::common::GetAbsolutePath;
using onboard::LidarFrameMessage;

bool LidarSegmentationComponent::Init() {
    // hook: Apollo License Verification: v_apollo_park
    LidarSegmentationComponentConfig comp_config;
    if (!GetProtoConfig(&comp_config)) {
        AERROR << "Get LidarSegmentationComponentConfig file failed";
        return false;
    }
    AINFO << "LidarSegmentation Component Configs: " << comp_config.DebugString();

    output_channel_name_ = comp_config.output_channel_name();
    writer_ = node_->CreateWriter<onboard::LidarFrameMessage>(output_channel_name_);

    // segmentor init
    auto plugin_param = comp_config.plugin_param();
    std::string segmentor_name = plugin_param.name();
    BaseLidarDetector* segmentor = BaseLidarDetectorRegisterer::GetInstanceByName(segmentor_name);
    CHECK_NOTNULL(segmentor);
    segmentor_.reset(segmentor);

    LidarDetectorInitOptions segmentr_init_options;
    segmentr_init_options.config_path = plugin_param.config_path();
    segmentr_init_options.config_file = plugin_param.config_file();
    ACHECK(segmentor_->Init(segmentr_init_options)) << "lidar segmentor init error";

    // semantic builder init
    SemanticBuilderInitOptions semantic_init_options;
    ACHECK(semantic_builder_.Init(semantic_init_options));

    AINFO << "LidarSegmentationComponent init successful.";
    return true;
}

bool LidarSegmentationComponent::Proc(const std::shared_ptr<LidarFrameMessage>& message) {
    // internal proc
    bool status = InternalProc(message);

    if (status) {
        writer_->Write(message);
        AINFO << "Send lidar segmentation output message.";
    }
    return status;
}

bool LidarSegmentationComponent::InternalProc(const std::shared_ptr<LidarFrameMessage>& message) {
    PERF_BLOCK("lidar_segmentation")
    LidarDetectorOptions segmentor_options;
    if (!segmentor_->Detect(segmentor_options, message->lidar_frame_.get())) {
        AERROR << "lidar segmentation Detect error.";
        return false;
    }
    PERF_BLOCK_END

    // semantic builder, get semantic and motion type
    PERF_BLOCK("semantic_builder")
    SemanticBuilderOptions semantic_options;
    if (!semantic_builder_.Build(semantic_options, message->lidar_frame_.get())) {
        AERROR << "Lidar segmentation, semantic builder error.";
        return false;
    }
    PERF_BLOCK_END

    return true;
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
