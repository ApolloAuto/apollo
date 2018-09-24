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
#ifndef PERCEPTION_FUSION_BASE_BASE_INIT_OPTIONS_H_
#define PERCEPTION_FUSION_BASE_BASE_INIT_OPTIONS_H_

#include <assert.h>
#include <string>

#include "modules/perception/lib/config_manager/config_manager.h"

namespace apollo {
namespace perception {
namespace fusion {

struct BaseInitOptions {
  std::string root_dir;
  std::string conf_file;
};

bool GetFusionInitOptions(const std::string& module_name,
                          BaseInitOptions* options);

}  // namespace fusion
}  // namespace perception
}  // namespace apollo

#endif  // PERCEPTION_FUSION_BASE_BASE_INIT_OPTIONS_H_
