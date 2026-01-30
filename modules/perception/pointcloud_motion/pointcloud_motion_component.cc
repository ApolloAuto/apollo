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

#include "modules/perception/pointcloud_motion/pointcloud_motion_component.h"

#include "cyber/profiler/profiler.h"

namespace apollo {
namespace perception {
namespace lidar {

bool PointCloudMotionComponent::Init() {
    PointCloudMotionComponentConfig comp_config;
    if (!GetProtoConfig(&comp_config)) {
        AERROR << "Get PointCloudMotionComponentConfig file failed";
        return false;
    }
    AINFO << "PointCloud Motion Component Configs: " << comp_config.DebugString();

    // writer
    output_channel_name_ = comp_config.output_channel_name();
    writer_ = node_->CreateWriter<LidarFrameMessage>(output_channel_name_);

    // init parser
    auto plugin_param = comp_config.plugin_param();
    std::string parser_name = plugin_param.name();
    BaseMotionParser* parser = BaseMotionParserRegisterer::GetInstanceByName(parser_name);
    CHECK_NOTNULL(parser);
    parser_.reset(parser);
    PointCloudParserInitOptions init_options;
    init_options.config_path = plugin_param.config_path();
    init_options.config_file = plugin_param.config_file();
    ACHECK(parser_->Init(init_options)) << "PointCloud parser init error";

    AINFO << "Successfully init pointcloud motion component.";
    return true;
}

bool PointCloudMotionComponent::Proc(const std::shared_ptr<LidarFrameMessage>& message) {
    PERF_FUNCTION()
    // internal proc
    bool status = InternalProc(message);

    if (status) {
        writer_->Write(message);
        AINFO << "Send PointCloud Motion message.";
    }
    return status;
}

bool PointCloudMotionComponent::InternalProc(const std::shared_ptr<LidarFrameMessage>& in_message) {
    // parser
    PERF_BLOCK("pointcloud_motion")
    PointCloudParserOptions parser_options;
    if (!parser_->Parse(parser_options, in_message->lidar_frame_.get())) {
        AERROR << "PointCloud motion error!";
        return false;
    }
    PERF_BLOCK_END

    return true;
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
