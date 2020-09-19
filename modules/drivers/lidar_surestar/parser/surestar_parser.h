/******************************************************************************
 * copyright 2020 The Apollo Authors. All Rights Reserved.
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

#ifndef SURESTAR_INCLUDE_SURESTAR_PARSER_SURESTAR_PARSER_H
#define SURESTAR_INCLUDE_SURESTAR_PARSER_SURESTAR_PARSER_H

#include <errno.h>
#include <stdint.h>

#include <map>
#include <memory>
#include <set>
#include <string>
#include <boost/format.hpp>

#include "modules/drivers/lidar_surestar/lib/calibration.h"
#include "modules/drivers/lidar_surestar/lib/const_variables.h"
#include "modules/drivers/lidar_surestar/lib/data_type.h"
#include "modules/drivers/lidar_surestar/proto/lidars_filter_config.pb.h"
#include "modules/drivers/lidar_surestar/proto/sensor_surestar.pb.h"
#include "modules/drivers/lidar_surestar/proto/sensor_surestar_conf.pb.h"
#include "modules/drivers/proto/pointcloud.pb.h"

namespace apollo {
namespace drivers {
namespace surestar {

/** \brief surestar data conversion class */
class SurestarParser {
 public:
  SurestarParser() {}
  explicit SurestarParser(
      const apollo::drivers::surestar::SurestarConfig& config);
  virtual ~SurestarParser() {}

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
      const std::shared_ptr<apollo::drivers::Surestar::SurestarScan const>&
          scan_msg,
      const std::shared_ptr<apollo::drivers::PointCloud>& out_msg) = 0;
  virtual void setup();
  virtual uint32_t GetPointSize() = 0;
  // order point cloud fod IDL by surestar model
  virtual void order(
      const std::shared_ptr<apollo::drivers::PointCloud>& cloud) = 0;

  const Calibration& get_calibration() { return _calibration; }
  double get_last_timestamp() { return _last_time_stamp; }

 protected:
  const float (*_inner_time)[12][32];
  /**
   * \brief Calibration file
   */
  Calibration _calibration;
  float _sin_rot_table[ROTATION_MAX_UNITS];
  float _cos_rot_table[ROTATION_MAX_UNITS];
  apollo::drivers::surestar::SurestarConfig _config;
  std::set<std::string> _filter_set;
  int _filter_grading;
  // Last surestar packet time stamp. (Full time)
  double _last_time_stamp;
  bool _need_two_pt_correction;
  uint32_t point_index_ = 0;
  Mode _mode;

  apollo::drivers::PointXYZIT get_nan_point(uint64_t timestamp);
  void init_angle_params(double view_direction, double view_width);
  /**
   * \brief Compute coords with the data in block
   *
   * @param tmp A two bytes union store the value of laser distance infomation
   * @param index The index of block
   */
  void compute_coords(const union RawDistance& raw_distance,
                      const LaserCorrection& corrections,
                      const uint16_t& rotation,
                      apollo::drivers::PointXYZIT* point);
  void compute_coords_beike(const float& raw_distance, const uint16_t rotation,
                            const int laserChannel,
                            apollo::drivers::PointXYZIT* point,
                            uint32_t* nan_pts);  // beike

  bool is_scan_valid(int rotation, float distance);

  /**
   * \brief Unpack surestar packet
   *
   */
  virtual void unpack(const apollo::drivers::Surestar::SurestarPacket& pkt,
                      const std::shared_ptr<apollo::drivers::PointCloud>& pc,
                      uint32_t* nan_pts) = 0;

  uint64_t get_gps_stamp(double current_stamp, double* previous_stamp,
                         uint64_t* gps_base_usec);

  virtual uint64_t get_timestamp(double base_time, float time_offset,
                                 uint16_t laser_block_id) = 0;

  void timestamp_check(double timestamp);

  void init_sin_cos_rot_table(float* sin_rot_table, float* cos_rot_table,
                              uint16_t rotation, float rotation_resolution);
};  // class SurestarParser

class Surestar16Parser : public SurestarParser {
 public:
  explicit Surestar16Parser(
      const apollo::drivers::surestar::SurestarConfig& config);
  ~Surestar16Parser() {}

  void generate_pointcloud(
      const std::shared_ptr<apollo::drivers::Surestar::SurestarScan const>&
          scan_msg,
      const std::shared_ptr<apollo::drivers::PointCloud>& out_msg);
  void order(const std::shared_ptr<apollo::drivers::PointCloud>& cloud);
  uint32_t GetPointSize() override;
  void setup() override;

 private:
  uint64_t get_timestamp(double base_time, float time_offset,
                         uint16_t laser_block_id);
  void unpack(const apollo::drivers::Surestar::SurestarPacket& pkt,
              const std::shared_ptr<apollo::drivers::PointCloud>& pc,
              uint32_t* nan_pts);
  // Previous surestar packet time stamp. (offset to the top hour)
  double _previous_packet_stamp;
  uint64_t _gps_base_usec;  // full time
  std::map<uint32_t, uint32_t> order_map_;
  uint32_t getOrderIndex(uint32_t index);
  void init_orderindex();
};  // class Surestar32Parser

class SurestarParserFactory {
 public:
  static SurestarParser* create_parser(
      const apollo::drivers::surestar::SurestarConfig& config);
};

}  // namespace  surestar
}  // namespace drivers
}  // namespace apollo

#endif  // SURESTAR_INCLUDE_SURESTAR_PARSER_SURESTAR_PARSER_H
