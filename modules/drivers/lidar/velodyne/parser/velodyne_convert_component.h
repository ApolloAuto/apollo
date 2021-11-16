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

#include <deque>
#include <memory>
#include <string>
#include <thread>

#include "modules/drivers/lidar/proto/velodyne.pb.h"
#include "modules/drivers/lidar/proto/velodyne_config.pb.h"

#include "cyber/base/concurrent_object_pool.h"
#include "cyber/cyber.h"
#include "modules/drivers/lidar/velodyne/parser/convert.h"

namespace apollo {
namespace drivers {
namespace velodyne {

using apollo::cyber::Component;
using apollo::cyber::Reader;
using apollo::cyber::Writer;
using apollo::cyber::base::CCObjectPool;
using apollo::drivers::PointCloud;
using apollo::drivers::velodyne::VelodyneScan;

class VelodyneConvertComponent : public Component<VelodyneScan> {
 public:
  bool Init() override;
  bool Proc(const std::shared_ptr<VelodyneScan>& scan_msg) override;

 private:
  std::shared_ptr<Writer<PointCloud>> writer_;
  std::unique_ptr<Convert> conv_ = nullptr;
  std::shared_ptr<CCObjectPool<PointCloud>> point_cloud_pool_ = nullptr;
  int pool_size_ = 8;
};

CYBER_REGISTER_COMPONENT(VelodyneConvertComponent)

}  // namespace velodyne
}  // namespace drivers
}  // namespace apollo
