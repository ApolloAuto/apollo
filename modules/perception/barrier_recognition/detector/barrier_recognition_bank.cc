/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/barrier_recognition/detector/barrier_recognition_bank.h"

#include "cyber/profiler/profiler.h"
#include "modules/perception/barrier_recognition/detector/proto/barrier_bank_config.pb.h"
#include "modules/perception/common/lidar/common/config_util.h"

namespace apollo {
namespace perception {
namespace lidar {

bool BarrierRecognizerBank::Init(const BarrierRecognizerInitOptions& options) {
    // get config
    std::string config_file = GetConfigFile(options.config_path, options.config_file);
    BarrierBankConfig config;
    ACHECK(cyber::common::GetProtoFromFile(config_file, &config));
    barrier_bank_map_.clear();
    for (int i = 0; i < config.plugins_size(); ++i) {
        const auto& plugin = config.plugins(i);
        const auto& name = plugin.name();
        std::shared_ptr<BaseBarrierRecognizer> recognizer =
            apollo::cyber::plugin_manager::PluginManager::Instance()
                ->CreateInstance<BaseBarrierRecognizer>(
                    ConfigUtil::GetFullClassName(name));
        if (!recognizer) {
            AINFO << "Failed to find barrier recognizer: " << name << ", skipped";
            continue;
        }
        BarrierRecognizerInitOptions option;
        option.config_path = plugin.config_path();
        option.config_file = plugin.config_file();
        if (!recognizer->Init(option)) {
            AINFO << "Failed to init barrier recognizer: " << name << ", skipped";
            continue;
        }
        barrier_bank_map_[name] = recognizer;
        AINFO << "Recognizer bank add: " << name;
    }
    return true;
}

bool BarrierRecognizerBank::Recognize(const BarrierRecognizerOptions& options, LidarFrame *frame, float& open_percent) {
    std::string barrier_type = options.name;
    if (barrier_bank_map_.find(barrier_type) == barrier_bank_map_.end()) {
        AERROR << "Failed to find barrier recognizer: " << barrier_type;
        return false;
    }
    if (!barrier_bank_map_[barrier_type]->Recognize(options, frame, open_percent)) {
        AINFO << "Failed to call barrier recognizer: " << barrier_type;
    }
    return true; 
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
