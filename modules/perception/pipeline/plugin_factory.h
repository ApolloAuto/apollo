/******************************************************************************
 * Copyright 2022 The Apollo Authors. All Rights Reserved.
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
#include <unordered_map>

#include "modules/common/util/factory.h"
#include "modules/perception/pipeline/plugin.h"

#include "modules/perception/pipeline/proto/pipeline_config.pb.h"

namespace apollo {
namespace perception {
namespace pipeline {

template <typename To, typename From, typename Deleter>
std::unique_ptr<To, Deleter>
    dynamic_unique_cast(std::unique_ptr<From, Deleter>&& p) {
  To* q = dynamic_cast<To*>(p.get());
  if (q) {
    std::unique_ptr<To, Deleter> res(q, std::move(p.get_deleter()));
    p.release();
    return res;
  }
  return std::unique_ptr<To, Deleter>(nullptr);
}

class PluginFactory {
 public:
  static void Init();

  static std::unique_ptr<Plugin>
        CreatePlugin(const PluginConfig& plugin_config);

 private:
  static apollo::common::util::Factory<
      PluginType, Plugin,
      Plugin *(*)(const PluginConfig& plugin_config),
      std::unordered_map<
          PluginType,
          Plugin *(*)(const PluginConfig& plugin_config),
          std::hash<int>>>
      plugin_factory_;
};

} // namespace pipeline
} // namespace perception
} // namespace apollo
