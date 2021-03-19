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
#pragma once

#include "gflags/gflags.h"

namespace apollo {
namespace perception {
namespace onboard {

DECLARE_bool(obs_enable_hdmap_input);
DECLARE_bool(obs_enable_visualization);
DECLARE_string(obs_screen_output_dir);
DECLARE_bool(obs_benchmark_mode);
DECLARE_bool(obs_save_fusion_supplement);
DECLARE_bool(start_visualizer);

}  // namespace onboard
}  // namespace perception
}  // namespace apollo
