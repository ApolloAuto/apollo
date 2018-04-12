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

/**
 * @file localization_params.h
 * @brief The class of LocalizationIntegParam
 */

#ifndef MODULES_LOCALIZATION_MSF_LOCALIZATION_PARAMS_H_
#define MODULES_LOCALIZATION_MSF_LOCALIZATION_PARAMS_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "modules/localization/proto/localization.pb.h"

/**
 * @namespace apollo::localization
 * @brief apollo::localization
 */
namespace apollo {
namespace localization {
namespace msf {

struct ImuToAntOffset {
  ImuToAntOffset() : offset_x(0.0), offset_y(0.0), offset_z(0.0),
      uncertainty_x(0.0), uncertainty_y(0.0), uncertainty_z(0.0) {}
  double offset_x;
  double offset_y;
  double offset_z;
  double uncertainty_x;
  double uncertainty_y;
  double uncertainty_z;
};

struct LocalizationIntegParam {
  // localization mode
  bool enable_lidar_localization = true;
  int gnss_mode = 0;

  // sins module
  bool is_ins_can_self_align = false;
  bool is_sins_align_with_vel = true;
  bool is_sins_state_check = false;
  double sins_state_span_time = 60.0;
  double sins_state_pos_std = 1.0;
  double vel_threshold_get_yaw = 5.0;
  bool integ_debug_log_flag = true;
  bool is_trans_gpstime_to_utctime = true;
  bool is_using_raw_gnsspos = true;

  // gnss module
  bool enable_ins_aid_rtk = false;
  ImuToAntOffset imu_to_ant_offset;
  bool gnss_debug_log_flag = true;

  // lidar module
  std::string map_path = "";
  std::string lidar_extrinsic_file = "";
  std::string lidar_height_file = "";
  double lidar_height_default = 1.7;
  bool lidar_debug_log_flag = true;
  int localization_mode = 2;
  int lidar_yaw_align_mode = 2;
  int lidar_filter_size = 17;
  double map_coverage_theshold = 0.8;
  double imu_lidar_max_delay_time = 0.4;
  int utm_zone_id = 50;
  bool is_lidar_unstable_reset = true;
  double unstable_reset_threshold = 0.08;
};

enum class LocalizationMeasureState { NOT_VALID = 0, NOT_STABLE, OK, VALID };

class LocalizationResult {
 public:
  LocalizationResult() : state_(LocalizationMeasureState::NOT_VALID) {}
  LocalizationResult(const LocalizationMeasureState& state,
                     const LocalizationEstimate& localiztion)
      : state_(state), localization_(localiztion) {}
  LocalizationMeasureState state() const {
    return state_;
  }
  LocalizationEstimate localization() const {
    return localization_;
  }

 private:
  LocalizationMeasureState state_;
  LocalizationEstimate localization_;
};

}  // namespace msf
}  // namespace localization
}  // namespace apollo

#endif  // MODULES_LOCALIZATION_MSF_LOCALIZATION_PARAMS_H_
