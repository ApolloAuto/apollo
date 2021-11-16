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

#pragma once

#include <memory>
#include <vector>

#include "cyber/base/concurrent_object_pool.h"
#include "cyber/cyber.h"

#include "modules/drivers/lidar/velodyne/compensator/compensator.h"

namespace apollo {
namespace drivers {
namespace velodyne {

using apollo::cyber::Component;
using apollo::cyber::Reader;
using apollo::cyber::Writer;
using apollo::cyber::base::CCObjectPool;
using apollo::drivers::PointCloud;

class CompensatorComponent : public Component<PointCloud> {
 public:
  bool Init() override;
  bool Proc(const std::shared_ptr<PointCloud>& point_cloud) override;

 private:
  std::unique_ptr<Compensator> compensator_ = nullptr;
  int pool_size_ = 8;
  int seq_ = 0;
  std::shared_ptr<Writer<PointCloud>> writer_ = nullptr;
  std::shared_ptr<CCObjectPool<PointCloud>> compensator_pool_ = nullptr;
};

CYBER_REGISTER_COMPONENT(CompensatorComponent)
}  // namespace velodyne
}  // namespace drivers
}  // namespace apollo
