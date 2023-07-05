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
#include "modules/perception/fusion/base/base_init_options.h"

#include "cyber/common/log.h"

#include "modules/perception/lib/config_manager/config_manager.h"

namespace apollo {
namespace perception {
namespace fusion {

bool GetFusionInitOptions(const std::string& module_name,
                          BaseInitOptions* options) {
  CHECK_NOTNULL(options);
  const lib::ModelConfig* model_config = nullptr;
  if (!lib::ConfigManager::Instance()->GetModelConfig(module_name,
                                                      &model_config)) {
    return false;
  }
  bool state = model_config->get_value("root_dir", &(options->root_dir)) &&
               model_config->get_value("config_file", &(options->conf_file));
  return state;
}

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
