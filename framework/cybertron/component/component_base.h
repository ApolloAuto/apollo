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

#ifndef CYBERTRON_COMPONENT_COMPONENT_BASE_H_
#define CYBERTRON_COMPONENT_COMPONENT_BASE_H_

#include <memory>
#include <string>
#include <vector>

#include "cybertron/class_loader/class_loader.h"
#include "cybertron/common/environment.h"
#include "cybertron/common/file.h"
#include "cybertron/node/node.h"
#include "cybertron/proto/component_config.pb.h"

namespace apollo {
namespace cybertron {

using apollo::cybertron::proto::ComponentConfig;
using apollo::cybertron::proto::TimerComponentConfig;

class ComponentBase : public std::enable_shared_from_this<ComponentBase> {
 public:
  template <typename M>
  using Reader = cybertron::Reader<M>;

  virtual ~ComponentBase() {}

  virtual bool Initialize(const ComponentConfig& config) { return false; }
  virtual bool Initialize(const TimerComponentConfig& config) { return false; }

  template <typename T>
  bool GetProtoConfig(T* config) {
    return common::GetProtoFromFile(config_file_path_, config);
  }

 protected:
  virtual bool Init() = 0;
  const std::string& ConfigFilePath() const { return config_file_path_; }
  void SetConfigFilePath(const ComponentConfig& config) {
    if (config.has_config_file_path()) {
      if (config.config_file_path()[0] != '/') {
        config_file_path_ = common::GetAbsolutePath(common::ModuleRoot(),
                                                    config.config_file_path());
      } else {
        config_file_path_ = config.config_file_path();
      }
    }
  }

  std::string config_file_path_ = "";
  std::vector<std::shared_ptr<ReaderBase>> readers;
};

}  // namespace cybertron
}  // namespace apollo

#endif  // CYBERTRON_COMPONENT_COMPONENT_BASE_H_
