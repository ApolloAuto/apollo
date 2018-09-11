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

// #include <nodelet/nodelet.h>
// #include <pluginlib/class_list_macros.h>
// #include <ros/ros.h>
// #include <boost/thread.hpp>
#include <string>
#include <thread>
#include <memory>

#include <cybertron/cybertron.h>

#include "modules/drivers/velodyne/proto/velodyne.pb.h"
#include "modules/drivers/velodyne/proto/config.pb.h"
#include "modules/drivers/velodyne/driver/driver.h"

namespace apollo {
namespace drivers {
namespace velodyne {

using apollo::drivers::velodyne::VelodyneScan;
using apollo::drivers::velodyne::config::Config;
using apollo::cybertron::Component;
using apollo::cybertron::Writer;
using apollo::cybertron::Reader;

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

