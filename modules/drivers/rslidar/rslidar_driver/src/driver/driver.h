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

#ifndef RSLIDAR_DRIVER_H
#define RSLIDAR_DRIVER_H

#include <ros/ros.h>
#include <string>

#include "rslidar_driver/socket_input.h"
#include "rslidar_msgs/rslidarScan.h"

namespace apollo {
namespace drivers {
namespace rslidar {

// configuration parameters
struct Config {
  Config()
      : npackets(0), rpm(0.0), msop_data_port(0), difop_data_port(0) {}
  std::string frame_id;  ///< tf frame ID
  std::string model;     ///< device model name
  std::string topic;
  int npackets;  ///< number of packets to collect
  double rpm;    ///< device rotation rate (RPMs)
  int msop_data_port;
  int difop_data_port;
};

class RslidarDriver {
 public:
  RslidarDriver();
  virtual ~RslidarDriver() {}

  virtual bool poll(void) = 0;
  virtual void init(ros::NodeHandle &node) = 0;

 protected:
  Config config_;
  boost::shared_ptr<Input> input_;
  ros::Publisher output_;
  std::string topic_;

  uint64_t basetime_;
  uint32_t last_gps_time_;
  int poll_standard(rslidar_msgs::rslidarScanPtr &scan);
  bool set_base_time();
  void set_base_time_from_nmea_time(const NMEATimePtr &nmea_time,
                                    uint64_t &basetime);
  void update_gps_top_hour(unsigned int current_time);
};

class Rslidar64Driver : public RslidarDriver {
 public:
  explicit Rslidar64Driver(const Config &config);
  virtual ~Rslidar64Driver() {}

  void init(ros::NodeHandle &node);
  bool poll(void);

 private:
};

class Rslidar32Driver : public RslidarDriver {
public:
    explicit Rslidar32Driver(const Config &config);
    virtual ~Rslidar32Driver() {}
    void init(ros::NodeHandle &node);
    bool poll(void);
    void poll_positioning_packet();
private:
  std::shared_ptr<Input> positioning_input_;
};

class Rslidar16Driver : public RslidarDriver {
 public:
  explicit Rslidar16Driver(const Config &config);
  virtual ~Rslidar16Driver() {}

  void init(ros::NodeHandle &node);
  bool poll(void);
  void poll_positioning_packet();

 private:
  std::shared_ptr<Input> positioning_input_;
};

class RslidarDriverFactory {
 public:
  static RslidarDriver *create_driver(ros::NodeHandle private_nh);
};

}  // namespace rslidar
}  // namespace drivers
}  // namespace apollo

#endif  
