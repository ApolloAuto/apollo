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

#ifndef ONBOARD_DRIVERS_SUTENG_INCLUDE_SUTENG_COMPONENT_COMPONENT_DRIVER_H
#define ONBOARD_DRIVERS_SUTENG_INCLUDE_SUTENG_COMPONENT_COMPONENT_DRIVER_H

#include <memory>
#include <string>
#include <thread>

#include "cyber/cyber.h"
#include "modules/drivers/lidar/lidar_robosense/include/driver/driver.h"
#include "modules/drivers/lidar/lidar_robosense/proto/sensor_suteng.pb.h"

namespace autobot {
namespace drivers {
namespace robosense {
using apollo::cyber::Component;
class CompRoboDriver : public Component<> {
 public:
  ~CompRoboDriver() {
    if (device_thread_->joinable()) {
      device_thread_->join();
    }
  }
  bool Init() {
    // read config file
    cybertron::proto::SutengConfig config;
    if (!apollo::cyber::common::GetProtoFromFile(config_file_path_, &config)) {
      AERROR << "Failed to load config file";
      return false;
    }
    AINFO << "config:" << config.DebugString();

    // set default main frame
    if (!config.has_main_frame() && config.frame_id() == "velodyne16_back") {
      config.set_main_frame(true);
    }

    RobosenseDriver *driver =
        robosense::RobosenseDriverFactory::create_driver(config);
    if (driver == nullptr) {
      return false;
    }
    writer_ = node_->CreateWriter<adu::common::sensor::suteng::SutengScan>(
        config.scan_channel());

    dvr_.reset(driver);
    dvr_->init();
    // spawn device poll thread
    runing_ = true;
    device_thread_ = std::shared_ptr<std::thread>(
        new std::thread(std::bind(&CompRoboDriver::device_poll, this)));
    device_thread_->detach();
    AINFO << "CompRoboDriver Init SUCC"
          << ", frame_id:" << config.frame_id();
    return true;
  }

 private:
  void device_poll() {
    while (!apollo::cyber::IsShutdown()) {
      std::shared_ptr<adu::common::sensor::suteng::SutengScan> scan(
          new adu::common::sensor::suteng::SutengScan);
      if (dvr_->poll(scan)) {
        writer_->Write(scan);
      } else {
        AWARN << "device poll failed";
      }
    }
    AERROR << "CompVelodyneDriver thread exit";
    runing_ = false;
  }

  // variable
  std::shared_ptr<RobosenseDriver> dvr_;
  volatile bool runing_;  ///< device thread is running
  uint32_t seq_ = 0;
  std::shared_ptr<std::thread> device_thread_;
  std::shared_ptr<
      apollo::cyber::Writer<adu::common::sensor::suteng::SutengScan>>
      writer_;
};

CYBER_REGISTER_COMPONENT(CompRoboDriver);
}  // namespace robosense
}  // namespace drivers
}  // namespace autobot

#endif  // ONBOARD_DRIVERS_SUTENG_INCLUDE_SUTENG_COMPONENT_COMPONENT_DRIVER_H
