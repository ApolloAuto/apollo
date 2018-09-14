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
#ifndef MODULES_DRIVERS_VELODYNE_DRIVER_VELODYNE_DRIVER_COMPONENT_H_
#define MODULES_DRIVERS_VELODYNE_DRIVER_VELODYNE_DRIVER_COMPONENT_H_

#include <memory>
#include <string>
#include <thread>

#include "cybertron/cybertron.h"

#include "modules/drivers/velodyne/driver/driver.h"
#include "modules/drivers/velodyne/proto/config.pb.h"
#include "modules/drivers/velodyne/proto/velodyne.pb.h"

namespace apollo {
namespace drivers {
namespace velodyne {

using apollo::cybertron::Component;
using apollo::cybertron::Reader;
using apollo::cybertron::Writer;
using apollo::drivers::velodyne::VelodyneScan;
using apollo::drivers::velodyne::config::Config;

class VelodyneDriverComponent : public Component<> {
 public:
  bool Init() override;

 private:
  void device_poll();
  volatile bool runing_;  ///< device thread is running
  uint32_t seq_ = 0;
  std::shared_ptr<std::thread> device_thread_;
  std::shared_ptr<VelodyneDriver> dvr_;  ///< driver implementation class
  std::shared_ptr<apollo::cybertron::Writer<VelodyneScan>> writer_;
};

CYBERTRON_REGISTER_COMPONENT(VelodyneDriverComponent)

}  // namespace velodyne
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_DRIVERS_VELODYNE_DRIVER_VELODYNE_DRIVER_COMPONENT_H_
