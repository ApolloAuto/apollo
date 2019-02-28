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
#include "modules/perception/lidar/lib/scene_manager/scene_manager.h"

#include "cyber/common/file.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/lidar/lib/scene_manager/proto/scene_manager_config.pb.h"
#include "modules/perception/lidar/lib/scene_manager/scene_service.h"
#include "modules/perception/proto/perception_config_schema.pb.h"

namespace apollo {
namespace perception {
namespace lidar {

using cyber::common::GetAbsolutePath;

bool SceneManager::InitInternal(const SceneManagerInitOptions& options) {
  if (initialized_) {
    return true;
  }
  auto config_manager = lib::ConfigManager::Instance();
  const lib::ModelConfig* model_config = nullptr;
  CHECK(config_manager->GetModelConfig(Name(), &model_config));
  const std::string work_root = config_manager->work_root();
  std::string config_file;
  std::string root_path;
  CHECK(model_config->get_value("root_path", &root_path));
  config_file = GetAbsolutePath(work_root, root_path);
  config_file = GetAbsolutePath(config_file, "scene_manager.conf");
  SceneManagerConfig config;
  CHECK(cyber::common::GetProtoFromFile(config_file, &config));
  services_.clear();
  for (int i = 0; i < config.service_name_size(); ++i) {
    const auto& name = config.service_name(i);
    SceneServicePtr service(SceneServiceRegisterer::GetInstanceByName(name));
    if (service == nullptr) {
      AINFO << "Failed to find scene service: " << name << ", skipped";
      continue;
    }
    if (!service->Init()) {
      AINFO << "Failed to init scene service: " << name << ", skipped";
      continue;
    }
    services_.emplace(name, service);
    AINFO << "Scene manager add service: " << name;
  }
  initialized_ = true;
  return true;
}

bool SceneManager::Init(const SceneManagerInitOptions& options) {
  std::lock_guard<std::mutex> lock(mutex_);
  bool status = InitInternal(options);
  return status;
}

bool SceneManager::Reset(const SceneManagerInitOptions& options) {
  std::lock_guard<std::mutex> lock(mutex_);
  initialized_ = false;
  bool status = InitInternal(options);
  return status;
}

SceneServicePtr SceneManager::Service(const std::string& name) {
  auto iter = services_.find(name);
  if (iter == services_.end()) {
    return nullptr;
  }
  return iter->second;
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
