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

#include <map>
#include <string>

#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/common/lidar/scene_manager/scene_service.h"
#include "modules/perception/common/util.h"
#include "modules/perception/common/lib/interface/base_init_options.h"

namespace apollo {
namespace perception {
namespace lidar {

using apollo::perception::BaseInitOptions;

struct SceneManagerInitOptions : public BaseInitOptions {};

class SceneManager {
 public:
  static SceneManager& Instance() {
    static SceneManager manager;
    return manager;
  }
  ~SceneManager() = default;
  // @brief: initialize scene manager with lock
  // @param [in]: initialization options
  // @return: status
  bool Init();
  // @brief: get service given name
  // @param [in]: service name
  // @return: service pointer
  SceneServicePtr Service(const std::string& name);
  // @brief: get name of this class
  // @return: name
  std::string Name() const { return "SceneManager"; }
  // @brief: reset scene manager
  // @param [in]: initialization options
  // @return: status
  bool Reset(
      const SceneManagerInitOptions& options = SceneManagerInitOptions());

  int GetServiceNum() const { return static_cast<int>(services_.size()); }

 protected:
  SceneManager() = default;
  // @brief: initialize scene manager
  // @param [in]: initialization options
  // @return: status
  bool InitInternal(
      const SceneManagerInitOptions& options = SceneManagerInitOptions());

 protected:
  std::map<std::string, SceneServicePtr> services_;
  bool initialized_ = false;
  std::mutex mutex_;
};

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
