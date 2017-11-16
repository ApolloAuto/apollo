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
 * @file localization_integ.h
 * @brief The class of LocalizationInteg
 */

#ifndef MODULES_LOCALIZATION_MSF_LOCALIZATION_INTEG_H_
#define MODULES_LOCALIZATION_MSF_LOCALIZATION_INTEG_H_

#include <sensor_msgs/PointCloud2.h>
#include "modules/drivers/gnss/proto/gnss_best_pose.pb.h"
#include "modules/drivers/gnss/proto/gnss_raw_observation.pb.h"
#include "modules/drivers/gnss/proto/imu.pb.h"
#include "modules/localization/proto/gnss_pnt_result.pb.h"
#include "modules/localization/proto/imu.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/localization/proto/measure.pb.h"
#include "modules/localization/proto/sins_pva.pb.h"

/**
 * @namespace apollo::localization
 * @brief apollo::localization
 */
namespace apollo {
namespace localization {

typedef apollo::drivers::gnss::EpochObservation EpochObservation;
typedef apollo::drivers::gnss::GnssEphemeris GnssEphemeris;
typedef drivers::gnss::GnssBestPose GnssBestPose;

enum class LocalizaitonMeasureState { NOT_VALID = 0, NOT_STABLE, OK };

enum class LocalizationErrorCode {
  INTEG_ERROR = 0,
  LIDAR_ERROR,
  GNSS_ERROR,
  OK
};

class LocalizationState {
public:
  LocalizationState() : error_code_(LocalizationErrorCode::OK), error_msg_("") {}
  
  LocalizationState(LocalizationErrorCode code, const std::string &msg) 
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

struct ImuToAntOffset {
  double offset_x;
  double offset_y;
  double offset_z;
  double uncertainty_x;
  double uncertainty_y;
  double uncertainty_z;
};

struct LocalizationIntegParam {
  LocalizationIntegParam() {
    is_using_raw_gnsspos = true;
    is_use_visualize = false;
    lidar_filter_size = 11;
    lidar_thread_num = 2;
  }

  // integration module
  bool is_ins_can_self_align;
  bool is_sins_align_with_vel;
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
  bool lidar_debug_log_flag;
  int localization_mode;
  int lidar_filter_size;
  int lidar_thread_num;
  double map_coverage_theshold;
  double imu_lidar_max_delay_time;

  // common
  int utm_zone_id;
  double imu_rate;

  bool is_use_visualize;
};

class MeasureRepublishProcess;
class LocalizationIntegProcess;
class LocalizationGnssProcess;
class LocalizationLidarProcess;

/**
 * @class LocalizationInteg
 *
 * @brief interface of msf localization
 */

class LocalizationInteg {
 public:
  LocalizationInteg();
  ~LocalizationInteg();
  // Initialization.
  LocalizationState Init(const LocalizationIntegParam& params);

  // Lidar pcd process.
  void PcdProcess(const sensor_msgs::PointCloud2& message);
  // Raw Imu process.
  void CorrectedImuProcess(const Imu& imu_msg);
  void RawImuProcess(const drivers::gnss::Imu& imu_msg);
  // Gnss Info process.
  void RawObservationProcess(const EpochObservation& raw_obs_msg);
  void RawEphemerisProcess(const GnssEphemeris& gnss_orbit_msg);
  // gnss best pose process
  void GnssBestPoseProcess(const GnssBestPose& bestgnsspos_msg);

  void GetLidarMeasure(LocalizaitonMeasureState& state,
                       IntegMeasure& lidar_measure);

  void GetIntegMeasure(LocalizaitonMeasureState& state, IntegSinsPva& sins_pva,
                       LocalizationEstimate& integ_localization);

  void GetGnssMeasure(LocalizaitonMeasureState& state,
                      IntegMeasure& gnss_measure);

 private:
  MeasureRepublishProcess* republish_process_;
  LocalizationIntegProcess* integ_process_;
  LocalizationGnssProcess* gnss_process_;
  LocalizationLidarProcess* lidar_process_;

  LocalizaitonMeasureState lidar_measure_state_;
  IntegMeasure lidar_measure_;

  LocalizaitonMeasureState integ_measure_state_;
  LocalizationEstimate integ_localization_;
  IntegSinsPva integ_sins_pva_;

  LocalizaitonMeasureState gnss_measure_state_;
  IntegMeasure gnss_measure_;
  bool is_use_gnss_bestpose_;
};

}  // namespace localization
}  // namespace apollo

#endif // MODULES_LOCALIZATION_MSF_LOCALIZATION_IMU_PROCESS_H_