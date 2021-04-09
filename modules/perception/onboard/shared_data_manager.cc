/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/onboard/shared_data_manager.h"

#include "modules/common/log.h"

namespace apollo {
namespace perception {

bool SharedDataManager::Init(const DAGConfig::SharedDataConfig &data_config) {
  if (inited_) {
    AWARN << "SharedDataManager Init twice.";
    return true;
  }

  for (auto &proto : data_config.datas()) {
    SharedData *shared_data =
        SharedDataRegisterer::GetInstanceByName(proto.name());

    if (shared_data == nullptr) {
      AERROR << "failed to get SharedData instance: " << proto.name();
      return false;
    }
    if (!shared_data->Init()) {
      AERROR << "failed to Init SharedData. name: " << proto.name();
      return false;
    }

    auto result = shared_data_map_.emplace(
        shared_data->name(), std::unique_ptr<SharedData>(shared_data));

    if (!result.second) {
      AERROR << "duplicate SharedData: " << shared_data->name();
      return true;
    }
  }

  AINFO << "SharedDataManager load " << shared_data_map_.size()
        << " SharedData.";

  inited_ = true;
  return true;
}

SharedData *SharedDataManager::GetSharedData(const std::string &name) const {
  auto citer = shared_data_map_.find(name);
  if (citer == shared_data_map_.end()) {
    return nullptr;
  }
  return citer->second.get();
}

void SharedDataManager::Reset() {
  for (auto &pair : shared_data_map_) {
    pair.second->Reset();
  }
  AINFO << "reset all SharedData. nums: " << shared_data_map_.size();
}

void SharedDataManager::RemoveStaleData() {
  for (auto &shared_data : shared_data_map_) {
    shared_data.second->RemoveStaleData();
  }
}

}  // namespace perception
}  // namespace apollo
