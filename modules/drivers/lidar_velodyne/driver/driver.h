/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#ifndef MODULES_DRIVERS_LIDAR_VELODYNE_DRIVER_DRIVER_H_
#define MODULES_DRIVERS_LIDAR_VELODYNE_DRIVER_DRIVER_H_

#include <memory>
#include <string>

#include "ros/include/velodyne_msgs/VelodyneScanUnified.h"

#include "modules/drivers/lidar_velodyne/proto/velodyne_conf.pb.h"

#include "modules/drivers/lidar_velodyne/common/socket_input.h"

namespace apollo {
namespace drivers {
namespace lidar_velodyne {

constexpr int BLOCKS_PER_PACKET = 12;
constexpr int BLOCK_SIZE = 100;

class VelodyneDriver {
 public:
  VelodyneDriver();
  virtual ~VelodyneDriver() {}

  virtual bool poll(void) = 0;
  virtual void init() = 0;

 protected:
  virtual int poll_standard(
      velodyne_msgs::VelodyneScanUnifiedPtr &scan);  // NOLINT
  bool set_base_time();
  void set_base_time_from_nmea_time(NMEATimePtr nmea_time,
                                    uint64_t &basetime);  // NOLINT
  void update_gps_top_hour(unsigned int current_time);

  VelodyneConf config_;
  boost::shared_ptr<Input> input_;
  ros::Publisher output_;
  std::string topic_;
  uint64_t basetime_;
  uint32_t last_gps_time_;
};

class Velodyne64Driver : public VelodyneDriver {
 public:
  explicit Velodyne64Driver(const VelodyneConf &config);
  virtual ~Velodyne64Driver() {}

  void init();
  bool poll(void);

 private:
  bool check_angle(velodyne_msgs::VelodynePacket &packet);  // NOLINT
  int poll_standard_sync(
      velodyne_msgs::VelodyneScanUnifiedPtr &scan);  // NOLINT
};

class Velodyne32Driver : public VelodyneDriver {
 public:
  explicit Velodyne32Driver(const VelodyneConf &config);
  virtual ~Velodyne32Driver() {}
  void init();
  bool poll(void);
  void poll_positioning_packet();

 private:
  std::shared_ptr<Input> positioning_input_;
};

class Velodyne16Driver : public VelodyneDriver {
 public:
  explicit Velodyne16Driver(const VelodyneConf &config);
  virtual ~Velodyne16Driver() {}

  void init();
  bool poll(void);
  void poll_positioning_packet();

 private:
  std::shared_ptr<Input> positioning_input_;
};

class VelodyneDriverFactory {
 public:
  static VelodyneDriver *create_driver(const VelodyneConf &velodyne_config);
};

}  // namespace lidar_velodyne
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_DRIVERS_LIDAR_VELODYNE_DRIVER_DRIVER_H_
