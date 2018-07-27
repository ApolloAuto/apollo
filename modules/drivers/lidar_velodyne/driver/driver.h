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

#ifndef MODULES_DRIVERS_VELODYN_DRIVER_DRIVER_H_
#define MODULES_DRIVERS_VELODYN_DRIVER_DRIVER_H_

#include <string>

#include "ros/include/velodyne_msgs/VelodyneScanUnified.h"
#include "ros/ros.h"

#include "modules/drivers/lidar_velodyne/proto/velodyne_conf.pb.h"

#include "modules/drivers/lidar_velodyne/common/socket_input.h"

namespace apollo {
namespace drivers {
namespace lidar_velodyne {

class VelodyneDriver {
 public:
  VelodyneDriver();
  virtual ~VelodyneDriver() {}

  virtual bool poll(velodyne_msgs::VelodyneScanUnifiedPtr scan) = 0;
  virtual bool init() = 0;

 protected:
  VelodyneConf config_;
  boost::shared_ptr<Input> input_;
  ros::Publisher output_;
  std::string topic_;

  uint64_t basetime_;
  uint32_t last_gps_time_;
  int poll_standard(velodyne_msgs::VelodyneScanUnifiedPtr scan);
  void set_base_time_from_nmea_time(const NMEATimePtr& nmea_time,
                                    uint64_t* basetime);
  void update_gps_top_hour(unsigned int current_time);
};

class Velodyne64Driver : public VelodyneDriver {
 public:
  explicit Velodyne64Driver(const VelodyneConf& conf);
  virtual ~Velodyne64Driver() {}

  bool init() override;
  bool poll(velodyne_msgs::VelodyneScanUnifiedPtr scan) override;

 private:
};

class Velodyne16Driver : public VelodyneDriver {
 public:
  explicit Velodyne16Driver(const VelodyneConf& conf);
  virtual ~Velodyne16Driver() {}

  bool init() override;
  bool poll(velodyne_msgs::VelodyneScanUnifiedPtr scan) override;
  void poll_positioning_packet();

 private:
  boost::shared_ptr<Input> positioning_input_;
};

class VelodyneDriverFactory {
 public:
  static VelodyneDriver* create_driver(const VelodyneConf& conf);
};

}  // namespace lidar_velodyne
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_DRIVERS_VELODYN_DRIVER_DRIVER_H__
