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

#ifndef MODULES_PERCEPTION_ONBOARD_SHARED_DATA_MANAGER_H_
#define MODULES_PERCEPTION_ONBOARD_SHARED_DATA_MANAGER_H_

#include <memory>
#include <string>
#include <unordered_map>

#include "modules/perception/onboard/proto/dag_config.pb.h"

#include "modules/common/macro.h"
#include "modules/perception/onboard/shared_data.h"

namespace apollo {
namespace perception {

class SharedDataManager {
 public:
  SharedDataManager() = default;
  ~SharedDataManager() = default;

  bool Init(const DAGConfig::SharedDataConfig &data_config);

  // thread-safe.
  SharedData *GetSharedData(const std::string &name) const;

  void Reset();

  void RemoveStaleData();

 private:
  std::unordered_map<std::string, std::unique_ptr<SharedData>> shared_data_map_;
  bool inited_ = false;
  DISALLOW_COPY_AND_ASSIGN(SharedDataManager);
};

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_ONBOARD_SHARED_DATA_MANAGER_H_
