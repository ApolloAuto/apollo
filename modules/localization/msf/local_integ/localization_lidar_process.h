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
 * @file localization_lidar_process.h
 * @brief The class of LocalizationLidarProcess
 */

#pragma once

#include <string>

#include "Eigen/Core"
#include "Eigen/Geometry"

// TODO(Localization): Fix the typo of "forecast".
#include "localization_msf/pose_forcast.h"
#include "localization_msf/sins_struct.h"
#include "modules/common/status/status.h"
#include "modules/localization/msf/local_integ/localization_lidar.h"
#include "modules/localization/msf/local_integ/localization_params.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/localization/proto/localization_status.pb.h"
#include "modules/localization/proto/measure.pb.h"
#include "modules/localization/proto/sins_pva.pb.h"

/**
 * @namespace apollo::localization
 * @brief apollo::localization
 */
namespace apollo {
namespace localization {
namespace msf {

enum class ForecastState { NOT_VALID = 0, INITIAL, INCREMENT };

enum class LidarState { NOT_VALID = 0, NOT_STABLE, OK };

enum class PredictLocationState {
  NOT_VALID = 0,
  INSPVA_ONLY,
  INSPVA_IMU,
  INSPVA_IMU_WHEEL
};

struct LidarHeight {
  LidarHeight() : height(0.0), height_var(0.0) {}
  double height;
  double height_var;
};

/**
 * @class LocalizationLidarProcess
 *
 * @brief process lidar msg for localization
 */
class LocalizationLidarProcess {
 public:
  typedef Eigen::Affine3d TransformD;
  typedef Eigen::Vector3d Vector3D;
  typedef Eigen::Matrix3d Matrix3D;

  LocalizationLidarProcess();
  ~LocalizationLidarProcess();

  // Initialization.
  apollo::common::Status Init(const LocalizationIntegParam &params);
  // Lidar pcd process and get result.
  void PcdProcess(const LidarFrame &lidar_frame);
  void GetResult(int *lidar_status, TransformD *location,
                 Matrix3D *covariance) const;
  int GetResult(LocalizationEstimate *lidar_local_msg);
  // Integrated navagation pva process.
  void IntegPvaProcess(const InsPva &sins_pva_msg);
  // Raw Imu process.
  void RawImuProcess(const ImuData &imu_msg);

 private:
  // Sub-functions for process.
  bool GetPredictPose(const double lidar_time, TransformD *inspva_pose,
                      ForecastState *forecast_state);
  bool CheckState();
  bool CheckDelta(const LidarFrame &frame, const TransformD &inspva_pose);
  void UpdateState(const int ret, const double time);

  // Load lidar-imu extrinsic parameter.
  bool LoadLidarExtrinsic(const std::string &file_path,
                          TransformD *lidar_extrinsic);
  // Load lidar height (the distance between lidar and ground).
  bool LoadLidarHeight(const std::string &file_path, LidarHeight *height);

  double ComputeDeltaYawLimit(const int64_t index_cur,
                              const int64_t index_stable,
                              const double limit_min, const double limit_max);

 private:
  // Lidar localization.
  LocalizationLidar *locator_;
  PoseForcast *pose_forecastor_;

  std::string map_path_;
  std::string lidar_extrinsic_file_;
  std::string lidar_height_file_;
  int localization_mode_ = 2;
  int yaw_align_mode_ = 2;
  int lidar_filter_size_ = 17;
  double delta_yaw_limit_ = 0.00436;
  double init_delta_yaw_limit_ = 0.02618;
  double compensate_pitch_roll_limit_ = 0.035;
  int utm_zone_id_ = 50;
  double map_coverage_theshold_ = 0.8;
  TransformD lidar_extrinsic_;
  LidarHeight lidar_height_;

  bool is_get_first_lidar_msg_ = false;
  TransformD cur_predict_location_;
  TransformD pre_predict_location_;
  Vector3D velocity_;
  TransformD pre_location_;
  TransformD location_;
  double pre_location_time_ = 0;

  // Information used to output.
  Matrix3D location_covariance_;
  LidarState lidar_status_;

  LocalLidarStatus local_lidar_status_ =
      LocalLidarStatus::MSF_LOCAL_LIDAR_UNDEFINED_STATUS;
  LocalLidarQuality local_lidar_quality_ =
      LocalLidarQuality::MSF_LOCAL_LIDAR_BAD;

  bool reinit_flag_ = false;

  // if use avx to accelerate lidar localization algorithm
  bool if_use_avx_ = false;

  // imu and lidar max delay time
  double imu_lidar_max_delay_time_ = 0.4;

  bool is_unstable_reset_ = false;
  int unstable_count_ = 0;
  double unstable_threshold_ = 0.3;

  int out_map_count_ = 0;

  /**@brief forecast integ pose, use to limit output of yaw */
  ForecastState forecast_integ_state_;
  int64_t forecast_timer_ = 0;

  static constexpr double DEG_TO_RAD = 0.017453292519943;
  static constexpr double DEG_TO_RAD2 = DEG_TO_RAD * DEG_TO_RAD;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace msf
}  // namespace localization
}  // namespace apollo
