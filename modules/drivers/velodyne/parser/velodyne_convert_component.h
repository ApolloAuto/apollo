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
#include "modules/drivers/velodyne/parser/convert.h"

namespace apollo {
namespace drivers {
namespace velodyne {

using apollo::drivers::velodyne::VelodyneScan;
using apollo::drivers::velodyne::config::Config;
using apollo::drivers::PointCloud;
using apollo::cybertron::Component;
using apollo::cybertron::Writer;
using apollo::cybertron::Reader;

class VelodyneConvertComponent : public Component<VelodyneScan> {
 public:
  bool Init() override;
  bool Proc(const std::shared_ptr<VelodyneScan>& scan_msg) override;
 private:
  std::shared_ptr<Writer<PointCloud>> writer_;
  std::unique_ptr<Convert> conv_ = nullptr;
  std::deque<std::shared_ptr<PointCloud>> point_cloud_deque_;
  int queue_size_ = 8;
  int index_ = 0;
};

CYBERTRON_REGISTER_COMPONENT(VelodyneConvertComponent)

}  // namespace velodyne
}  // namespace drivers
}  // namespace apollo

