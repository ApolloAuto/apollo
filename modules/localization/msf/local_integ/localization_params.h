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
// #include "modules/drivers/gnss/proto/gnss_best_pose.pb.h"
// #include "modules/drivers/gnss/proto/gnss_raw_observation.pb.h"
// #include "modules/drivers/gnss/proto/imu.pb.h"
// #include "modules/localization/proto/gnss_pnt_result.pb.h"
#include "modules/localization/proto/localization.pb.h"
// #include "modules/localization/proto/measure.pb.h"
// #include "modules/localization/proto/sins_pva.pb.h"

// #include <list>
// #include "localization_struct.h"
// #include "localization_gnss_pnt_result.h"
// #include "localization_gnss_raw_observation.h"

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
  LocalizationIntegParam() : is_ins_can_self_align(false),
      is_sins_align_with_vel(true), is_sins_state_check(false),
      sins_state_span_time(60.0), sins_state_pos_std(1.0),
      vel_threshold_get_yaw(5.0), integ_debug_log_flag(true),
      is_trans_gpstime_to_utctime(true), gnss_mode(0),
      is_using_raw_gnsspos(true), enable_ins_aid_rtk(false),
      enable_auto_save_eph_file(true), eph_buffer_path(""),
      imu_to_ant_offset(), gnss_debug_log_flag(true), map_path(""),
      lidar_extrinsic_file(""), lidar_height_file(""), 
      lidar_height_default(1.7), lidar_debug_log_flag(true),
      localization_mode(2), lidar_yaw_align_mode(1),
      lidar_filter_size(17), lidar_thread_num(1), map_coverage_theshold(0.8),
      imu_lidar_max_delay_time(0.4), utm_zone_id(50), imu_rate(1.0),
      enable_lidar_localization(true), is_use_visualize(false) {}

  // integration module
  bool is_ins_can_self_align;
  bool is_sins_align_with_vel;
  bool is_sins_state_check;
  double sins_state_span_time;
  double sins_state_pos_std;
  double vel_threshold_get_yaw;
  bool integ_debug_log_flag;
  bool is_trans_gpstime_to_utctime;
  int gnss_mode;
  bool is_using_raw_gnsspos;

  // gnss module
  bool enable_ins_aid_rtk;
  bool enable_auto_save_eph_file;

  std::string eph_buffer_path;
  ImuToAntOffset imu_to_ant_offset;
  bool gnss_debug_log_flag;

  // lidar module
  std::string map_path;
  std::string lidar_extrinsic_file;
  std::string lidar_height_file;
  double lidar_height_default;
  bool lidar_debug_log_flag;
  int localization_mode;
  int lidar_yaw_align_mode;
  int lidar_filter_size;
  int lidar_thread_num;
  double map_coverage_theshold;
  double imu_lidar_max_delay_time;

  // common
  int utm_zone_id;
  double imu_rate;
  bool enable_lidar_localization;

  bool is_use_visualize;
};

enum class LocalizationErrorCode {
  INTEG_ERROR = 0,
  LIDAR_ERROR,
  GNSS_ERROR,
  OK
};

class LocalizationState {
 public:
  LocalizationState()
      : error_code_(LocalizationErrorCode::OK), error_msg_("") {}

  LocalizationState(LocalizationErrorCode code, const std::string& msg)
      : error_code_(code), error_msg_(msg) {}

  static LocalizationState OK() {
    return LocalizationState();
  }

  LocalizationErrorCode error_code() const {
    return error_code_;
  }

  std::string error_msg() const {
    return error_msg_;
  }

  bool ok() {
    return error_code_ == LocalizationErrorCode::OK;
  }

 private:
  LocalizationErrorCode error_code_;
  std::string error_msg_;
};

struct LidarFrame {
  double measurement_time; // unix time
  std::vector<double> pt_xs;
  std::vector<double> pt_ys;
  std::vector<double> pt_zs;
  std::vector<unsigned char> intensities;
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

} // namespace msf
} // namespace localization
} // namespace apollo

#endif  // MODULES_LOCALIZATION_MSF_LOCALIZATION_PARAMS_H_
