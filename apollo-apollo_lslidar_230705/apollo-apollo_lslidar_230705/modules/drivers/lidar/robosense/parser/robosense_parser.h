/******************************************************************************
 * Copyright 2021 The Apollo Authors. All Rights Reserved.
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
#include <errno.h>
#include <stdint.h>

#include <map>
#include <memory>
#include <set>
#include <string>

#include "boost/format.hpp"

#include "modules/common_msgs/sensor_msgs/pointcloud.pb.h"
#include "modules/drivers/lidar/robosense/proto/lidars_filter_config.pb.h"
#include "modules/drivers/lidar/robosense/proto/sensor_suteng.pb.h"
#include "modules/drivers/lidar/robosense/proto/sensor_suteng_conf.pb.h"

#include "modules/drivers/lidar/robosense/lib/calibration.h"
#include "modules/drivers/lidar/robosense/lib/const_variables.h"
#include "modules/drivers/lidar/robosense/lib/data_type.h"

namespace apollo {
namespace drivers {
namespace robosense {

/** \brief suteng data conversion class */
class RobosenseParser {
 public:
  RobosenseParser() {}
  explicit RobosenseParser(const apollo::drivers::suteng::SutengConfig& config);
  virtual ~RobosenseParser() {}

  /** \brief Set up for data processing.
   *
   *  Perform initializations needed before data processing can
   *  begin:
   *
   *    - read device-specific angles calibration
   *
   *  @param private_nh private node handle for ROS parameters
   *  @returns 0 if successful;
   *           errno value for failure
   */
  virtual void generate_pointcloud(
      const std::shared_ptr<apollo::drivers::suteng::SutengScan const>&
          scan_msg,
      const std::shared_ptr<apollo::drivers::PointCloud>& out_msg) = 0;
  virtual void setup();
  virtual uint32_t GetPointSize() = 0;
  // order point cloud fod IDL by suteng model
  virtual void order(
      const std::shared_ptr<apollo::drivers::PointCloud>& cloud) = 0;

  const Calibration& get_calibration() { return calibration_; }
  double get_last_timestamp() { return last_time_stamp_; }

  // suteng
  float CalibIntensity(float intensity, int cal_idx, int distance,
                       float temper);
  float PixelToDistance(int pixel_value, int passage_way, float temper);

 protected:
  // suteng
  int TEMPERATURE_RANGE = 40;  // rs16  rs32-50
  int TEMPERATURE_MIN = 31;
  int EstimateTemperature(float temper);
  float compute_temperature(unsigned char bit1, unsigned char bit2);

  const float (*_inner_time)[12][32];
  /**
   * \brief Calibration file
   */
  Calibration calibration_;
  float sin_rot_table_[ROTATION_MAX_UNITS];
  float cos_rot_table_[ROTATION_MAX_UNITS];
  apollo::drivers::suteng::SutengConfig config_;
  std::set<std::string> filter_set_;
  int filter_grading_;
  // Last suteng packet time stamp. (Full time)
  double last_time_stamp_;
  bool need_two_pt_correction_;
  uint32_t point_index_ = 0;
  Mode mode_;

  apollo::drivers::PointXYZIT get_nan_point(uint64_t timestamp);
  void init_angle_params(double view_direction, double view_width);
  /**
   * \brief Compute coords with the data in block
   *
   * @param tmp A two bytes union store the value of laser distance infomation
   * @param index The index of block
   */
  bool is_scan_valid(int rotation, float distance);

  /**
   * \brief Unpack suteng packet
   *
   */
  //   virtual void unpack(const apollo::drivers::suteng::SutengPacket& pkt,
  //                       std::shared_ptr<apollo::drivers::PointCloud>& pc) =
  //                       0;

  uint64_t get_gps_stamp(double current_stamp, double* previous_stamp,
                         uint64_t* gps_base_usec);

  virtual uint64_t get_timestamp(double base_time, float time_offset,
                                 uint16_t laser_block_id) = 0;

  void timestamp_check(double timestamp);

  void init_sin_cos_rot_table(float* sin_rot_table, float* cos_rot_table,
                              uint16_t rotation, float rotation_resolution);
};  // class RobosenseParser

class Robosense16Parser : public RobosenseParser {
 public:
  explicit Robosense16Parser(
      const apollo::drivers::suteng::SutengConfig& config);
  ~Robosense16Parser() {}

  void generate_pointcloud(
      const std::shared_ptr<apollo::drivers::suteng::SutengScan const>&
          scan_msg,
      const std::shared_ptr<apollo::drivers::PointCloud>& out_msg);
  void order(const std::shared_ptr<apollo::drivers::PointCloud>& cloud);
  uint32_t GetPointSize() override;
  void setup() override;
  void init_setup();

 private:
  uint64_t get_timestamp(double base_time, float time_offset,
                         uint16_t laser_block_id);

  void unpack_robosense(
      const apollo::drivers::suteng::SutengPacket& pkt,
      const std::shared_ptr<apollo::drivers::PointCloud>& cloud,
      uint32_t* index);

  // Previous suteng packet time stamp. (offset to the top hour)
  double previous_packet_stamp_;
  uint64_t gps_base_usec_;  // full time
  std::map<uint32_t, uint32_t> order_map_;
  uint32_t getOrderIndex(uint32_t index);
  void init_orderindex();
  RslidarPic pic;

  // sutegn
  int temp_packet_num = 0;
  float temper = 31.f;

  static bool pkt_start;
  static uint64_t base_stamp;

  uint64_t first_pkt_stamp;
  uint64_t final_pkt_stamp;
  uint64_t last_pkt_stamp = 0;
};  // class Robosense32Parser

class RobosenseParserFactory {
 public:
  static RobosenseParser* create_parser(
      const apollo::drivers::suteng::SutengConfig& config);
};

}  // namespace robosense
}  // namespace drivers
}  // namespace apollo
