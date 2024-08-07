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

#include "modules/common_msgs/sensor_msgs/nano_radar.pb.h"
#include "modules/drivers/radar/nano_radar/proto/nano_radar_conf.pb.h"

#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace drivers {
namespace nano_radar {

using apollo::drivers::NanoRadar;

class RegionConfig401
    : public apollo::drivers::canbus::ProtocolData<NanoRadar> {
 public:
  static const uint32_t ID;
  RegionConfig401();
  ~RegionConfig401();
  /**
   * @brief get the data period
   * @return the value of data period
   */
  uint32_t GetPeriod() const override;

  /**
   * @brief update the data
   * @param data a pointer to the data to be updated
   */
  void UpdateData(uint8_t* data) override;

  /**
   * @brief reset the private variables
   */
  void Reset() override;

  RegionConfig401* set_collision_detection_coordinates_valid(bool valid);
  RegionConfig401* set_collision_detection_activation_valid(bool valid);
  RegionConfig401* set_region_max_output_number(uint8_t data);
  RegionConfig401* set_region_id(uint8_t datad);
  RegionConfig401* set_point1_longitude(double data);
  RegionConfig401* set_point1_lateral(double data);
  RegionConfig401* set_point2_longitude(double data);
  RegionConfig401* set_point2_lateral(double data);
  RegionConfig401* set_radar_conf(RadarConf radar_conf);
  RadarConf radar_conf();

  void set_collision_detection_coordinates_valid_p(uint8_t* data, bool valid);
  void set_collision_detection_activation_valid_p(uint8_t* data, bool valid);

  void set_region_max_output_number_p(uint8_t* data, uint8_t value);
  void set_region_id_p(uint8_t* data, uint8_t value);
  void set_point1_longitude_p(uint8_t* data, double value);
  void set_point1_lateral_p(uint8_t* data, double value);
  void set_point2_longitude_p(uint8_t* data, double value);
  void set_point2_lateral_p(uint8_t* data, double value);

 private:
  RadarConf radar_conf_;
};

}  // namespace nano_radar
}  // namespace drivers
}  // namespace apollo
