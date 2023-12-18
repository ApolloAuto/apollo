/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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
#include "modules/perception/common/onboard/common_flags/common_flags.h"

namespace apollo {
namespace perception {
namespace onboard {

DEFINE_bool(obs_enable_hdmap_input, true, "enable hdmap input for roi filter");
DEFINE_bool(obs_enable_visualization, false,
            "whether to send message for visualization");
DEFINE_string(obs_screen_output_dir, "./",
              "output dir. for saving visualization screenshots");
DEFINE_bool(obs_benchmark_mode, false,
            "whether open benchmark mode, default false");
DEFINE_bool(obs_save_fusion_supplement, false,
            "whether save fusion supplement data, default false");
DEFINE_bool(start_visualizer, false, "Whether to start visualizer");

}  // namespace onboard
}  // namespace perception
}  // namespace apollo
