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

#pragma once

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/drivers/proto/racobit_radar.pb.h"

namespace apollo {
namespace drivers {
namespace racobit_radar {

using apollo::drivers::RacobitRadar;
using apollo::drivers::racobit_radar::OutputType;
using apollo::drivers::racobit_radar::RadarConf;
using apollo::drivers::racobit_radar::RcsThreshold;

class RadarConfig200
    : public apollo::drivers::canbus::ProtocolData<RacobitRadar> {
 public:
  static const uint32_t ID;
  RadarConfig200();
  ~RadarConfig200();
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

  RadarConfig200* set_max_distance_valid(bool valid);
  RadarConfig200* set_sensor_id_valid(bool valid);
  RadarConfig200* set_radar_power_valid(bool valid);
  RadarConfig200* set_output_type_valid(bool valid);
  RadarConfig200* set_send_quality_valid(bool valid);
  RadarConfig200* set_send_ext_info_valid(bool valid);
  RadarConfig200* set_sort_index_valid(bool valid);
  RadarConfig200* set_store_in_nvm_valid(bool valid);
  RadarConfig200* set_ctrl_relay_valid(bool valid);
  RadarConfig200* set_rcs_threshold_valid(bool valid);

  RadarConfig200* set_max_distance(uint16_t data);
  RadarConfig200* set_sensor_id(uint8_t data);
  RadarConfig200* set_output_type(OutputType type);
  RadarConfig200* set_radar_power(uint8_t data);
  RadarConfig200* set_ctrl_relay(uint8_t data);
  RadarConfig200* set_send_ext_info(uint8_t data);
  RadarConfig200* set_send_quality(uint8_t data);
  RadarConfig200* set_sort_index(uint8_t data);
  RadarConfig200* set_store_in_nvm(uint8_t data);
  RadarConfig200* set_rcs_threshold(RcsThreshold rcs_theshold);
  RadarConfig200* set_radar_conf(RadarConf radar_conf);
  RadarConf radar_conf();

  void set_max_distance_valid_p(uint8_t* data, bool valid);
  void set_sensor_id_valid_p(uint8_t* data, bool valid);
  void set_radar_power_valid_p(uint8_t* data, bool valid);
  void set_output_type_valid_p(uint8_t* data, bool valid);
  void set_send_quality_valid_p(uint8_t* data, bool valid);
  void set_send_ext_info_valid_p(uint8_t* data, bool valid);
  void set_sort_index_valid_p(uint8_t* data, bool valid);
  void set_store_in_nvm_valid_p(uint8_t* data, bool valid);
  void set_ctrl_relay_valid_p(uint8_t* data, bool valid);
  void set_rcs_threshold_valid_p(uint8_t* data, bool valid);

  void set_max_distance_p(uint8_t* data, uint16_t value);
  void set_sensor_id_p(uint8_t* data, uint8_t value);
  void set_output_type_p(uint8_t* data, OutputType type);
  void set_radar_power_p(uint8_t* data, uint8_t value);
  void set_ctrl_relay_p(uint8_t* data, uint8_t value);
  void set_send_ext_info_p(uint8_t* data, uint8_t value);
  void set_send_quality_p(uint8_t* data, uint8_t value);
  void set_sort_index_p(uint8_t* data, uint8_t value);
  void set_store_in_nvm_p(uint8_t* data, uint8_t value);
  void set_rcs_threshold_p(uint8_t* data, RcsThreshold rcs_theshold);

 private:
  RadarConf radar_conf_;
};

}  // namespace racobit_radar
}  // namespace drivers
}  // namespace apollo
