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

#ifndef MODULES_DRIVERS_VELODYNE_PARSER_VELODYNE_COMPENSATOR_COMPONENT_H
#define MODULES_DRIVERS_VELODYNE_PARSER_VELODYNE_COMPENSATOR_COMPONENT_H 

#include <memory>

#include "cybertron/cybertron.h"

#include "modules/drivers/proto/pointcloud.pb.h"
#include "modules/drivers/velodyne/proto/velodyne.pb.h"
#include "modules/drivers/velodyne/parser/compensator.h"

namespace apollo {
namespace drivers {
namespace velodyne {

using apollo::drivers::velodyne::config::Config;
using apollo::drivers::PointCloud;
using apollo::cybertron::Component;
using apollo::cybertron::Writer;
using apollo::cybertron::Reader;

class VelodyneCompensatorComponent : public Component<PointCloud> {
 public:
  bool Init() override;
  bool Proc(
      const std::shared_ptr<PointCloud>& point_cloud) override;
 private:
  std::unique_ptr<Compensator> _compensator = nullptr;
  std::vector<std::shared_ptr<PointCloud>> compensator_deque_;
  int queue_size_ = 8;
  int index_ = 0;
  int seq_ = 0;
  std::shared_ptr<Writer<PointCloud>> writer_ = nullptr;
};

CYBERTRON_REGISTER_COMPONENT(VelodyneCompensatorComponent)
}
}
}  // namespace cybertron

#endif  // ONBOARD_DRIVERS_VELODYNE_INCLUDE_VELODYNE_COMPONENT_COMPONENT_COMPENSATOR_H
