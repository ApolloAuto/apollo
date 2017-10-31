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

#ifndef VELODYNE_DRIVER_H
#define VELODYNE_DRIVER_H

#include <ros/ros.h>
#include <string>

#include "velodyne_driver/socket_input.h"
#include "velodyne_msgs/VelodyneScanUnified.h"

namespace apollo {
namespace drivers {
namespace velodyne {

// configuration parameters
struct Config {
  std::string frame_id;  ///< tf frame ID
  std::string model;     ///< device model name
  std::string topic;
  int npackets;  ///< number of packets to collect
  double rpm;    ///< device rotation rate (RPMs)
  int firing_data_port;
  int positioning_data_port;
};

class VelodyneDriver {
 public:
  VelodyneDriver();
  ~VelodyneDriver() {}

  virtual bool poll(void) = 0;
  virtual void init(ros::NodeHandle &node) = 0;

 protected:
  Config config_;
  boost::shared_ptr<Input> input_;
  ros::Publisher output_;
  std::string topic_;

  uint64_t basetime_;
  uint32_t last_gps_time_;
  int poll_standard(velodyne_msgs::VelodyneScanUnifiedPtr &scan);
  bool set_base_time();
  void set_base_time_from_nmea_time(const NMEATimePtr &nmea_time,
                                    uint64_t &basetime);
  void update_gps_top_hour(unsigned int current_time);
};

class Velodyne64Driver : public VelodyneDriver {
 public:
  Velodyne64Driver(Config config);
  ~Velodyne64Driver() {}

  void init(ros::NodeHandle &node);
  bool poll(void);

 private:
};

class Velodyne16Driver : public VelodyneDriver {
 public:
  Velodyne16Driver(Config config);
  ~Velodyne16Driver() {}

  void init(ros::NodeHandle &node);
  bool poll(void);
  void poll_positioning_packet();

 private:
  boost::shared_ptr<Input> positioning_input_;
};
class VelodyneDriverFactory {
 public:
  static VelodyneDriver *create_driver(ros::NodeHandle private_nh);
};

}  // namespace velodyne
}  // namespace drivers
}  // namespace apollo

#endif  // VELODYNE_DRIVER_H__
