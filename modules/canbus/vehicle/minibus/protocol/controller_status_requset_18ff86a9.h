/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/canbus/proto/chassis_detail.pb.h"

#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace minibus {

class Controllerstatusrequset18ff86a9
    : public ::apollo::drivers::canbus::ProtocolData<
          ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Controllerstatusrequset18ff86a9();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  Controllerstatusrequset18ff86a9* set_sr_rtk_power(int sr_rtk_power);

  Controllerstatusrequset18ff86a9* set_sr_fullview_power(int sr_fullview_power);

  Controllerstatusrequset18ff86a9* set_sr_mobieye_power(int sr_mobieye_power);

  Controllerstatusrequset18ff86a9* set_sr_mmradar_power(int sr_mmradar_power);

  Controllerstatusrequset18ff86a9* set_sr_ultrasonicradar_power(
      int sr_ultrasonicradar_power);

  Controllerstatusrequset18ff86a9* set_sr_bsdradar_power(int sr_bsdradar_power);

  Controllerstatusrequset18ff86a9* set_sr_singlelindlidar_power(
      int sr_singlelindlidar_power);

  Controllerstatusrequset18ff86a9* set_sr_16lidar_power(int sr_16lidar_power);

  Controllerstatusrequset18ff86a9* set_sr_horn(
      Controller_status_requset_18ff86a9::Sr_hornType sr_horn);

  Controllerstatusrequset18ff86a9* set_sr_fluorescentlamps(
      int sr_fluorescentlamps);

  Controllerstatusrequset18ff86a9* set_sr_rooflight(int sr_rooflight);

  Controllerstatusrequset18ff86a9* set_sr_minillight(int sr_minillight);

  Controllerstatusrequset18ff86a9* set_sr_breaklight(
      Controller_status_requset_18ff86a9::Sr_breaklightType sr_breaklight);

  Controllerstatusrequset18ff86a9* set_sr_turnright(
      Controller_status_requset_18ff86a9::Sr_turnrightType sr_turnright);

  Controllerstatusrequset18ff86a9* set_sr_turnleft(
      Controller_status_requset_18ff86a9::Sr_turnleftType sr_turnleft);

  Controllerstatusrequset18ff86a9* set_sr_reversinglight(int sr_reversinglight);

  Controllerstatusrequset18ff86a9* set_sr_lowbeam(
      Controller_status_requset_18ff86a9::Sr_lowbeamType sr_lowbeam);

  Controllerstatusrequset18ff86a9* set_sr_fdoor(
      Controller_status_requset_18ff86a9::Sr_fdoorType sr_fdoor);

  Controllerstatusrequset18ff86a9* set_sr_gear_status(int sr_gear_status);

  Controllerstatusrequset18ff86a9* set_sr_vehiclemoveing_status(
      int sr_vehiclemoveing_status);

  Controllerstatusrequset18ff86a9* set_sr_drive_status(int sr_drive_status);

  Controllerstatusrequset18ff86a9* set_sr_inertialnavigation_status(
      int sr_inertialnavigation_status);

  Controllerstatusrequset18ff86a9* set_sr_rtk_status(int sr_rtk_status);

 private:
  void set_p_sr_rtk_power(uint8_t* data, int sr_rtk_power);

  void set_p_sr_fullview_power(uint8_t* data, int sr_fullview_power);

  void set_p_sr_mobieye_power(uint8_t* data, int sr_mobieye_power);

  void set_p_sr_mmradar_power(uint8_t* data, int sr_mmradar_power);

  void set_p_sr_ultrasonicradar_power(uint8_t* data,
                                      int sr_ultrasonicradar_power);

  void set_p_sr_bsdradar_power(uint8_t* data, int sr_bsdradar_power);

  void set_p_sr_singlelindlidar_power(uint8_t* data,
                                      int sr_singlelindlidar_power);

  void set_p_sr_16lidar_power(uint8_t* data, int sr_16lidar_power);

  void set_p_sr_horn(uint8_t* data,
                     Controller_status_requset_18ff86a9::Sr_hornType sr_horn);

  void set_p_sr_fluorescentlamps(uint8_t* data, int sr_fluorescentlamps);

  void set_p_sr_rooflight(uint8_t* data, int sr_rooflight);

  void set_p_sr_minillight(uint8_t* data, int sr_minillight);

  void set_p_sr_breaklight(
      uint8_t* data,
      Controller_status_requset_18ff86a9::Sr_breaklightType sr_breaklight);

  void set_p_sr_turnright(
      uint8_t* data,
      Controller_status_requset_18ff86a9::Sr_turnrightType sr_turnright);

  void set_p_sr_turnleft(
      uint8_t* data,
      Controller_status_requset_18ff86a9::Sr_turnleftType sr_turnleft);

  void set_p_sr_reversinglight(uint8_t* data, int sr_reversinglight);

  void set_p_sr_lowbeam(
      uint8_t* data,
      Controller_status_requset_18ff86a9::Sr_lowbeamType sr_lowbeam);

  void set_p_sr_fdoor(
      uint8_t* data, Controller_status_requset_18ff86a9::Sr_fdoorType sr_fdoor);

  void set_p_sr_gear_status(uint8_t* data, int sr_gear_status);

  void set_p_sr_vehiclemoveing_status(uint8_t* data,
                                      int sr_vehiclemoveing_status);

  void set_p_sr_drive_status(uint8_t* data, int sr_drive_status);

  void set_p_sr_inertialnavigation_status(uint8_t* data,
                                          int sr_inertialnavigation_status);

  void set_p_sr_rtk_status(uint8_t* data, int sr_rtk_status);

 private:
  int sr_rtk_power_;
  int sr_fullview_power_;
  int sr_mobieye_power_;
  int sr_mmradar_power_;
  int sr_ultrasonicradar_power_;
  int sr_bsdradar_power_;
  int sr_singlelindlidar_power_;
  int sr_16lidar_power_;
  Controller_status_requset_18ff86a9::Sr_hornType sr_horn_;
  int sr_fluorescentlamps_;
  int sr_rooflight_;
  int sr_minillight_;
  Controller_status_requset_18ff86a9::Sr_breaklightType sr_breaklight_;
  Controller_status_requset_18ff86a9::Sr_turnrightType sr_turnright_;
  Controller_status_requset_18ff86a9::Sr_turnleftType sr_turnleft_;
  int sr_reversinglight_;
  Controller_status_requset_18ff86a9::Sr_lowbeamType sr_lowbeam_;
  Controller_status_requset_18ff86a9::Sr_fdoorType sr_fdoor_;
  int sr_gear_status_;
  int sr_vehiclemoveing_status_;
  int sr_drive_status_;
  int sr_inertialnavigation_status_;
  int sr_rtk_status_;
};

}  // namespace minibus
}  // namespace canbus
}  // namespace apollo
