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
 * @file localization_lidar_process.h
 * @brief The class of LocalizationLidarProcess
 */

#ifndef MODULES_LOCALIZATION_MSF_LOCALIZATION_LIDAR_PROCESS_H_
#define MODULES_LOCALIZATION_MSF_LOCALIZATION_LIDAR_PROCESS_H_

#include <pthread.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <list>
#include <string>
#include <cstdint>
#include "modules/common/status/status.h"
#include "modules/localization/msf/local_integ/localization_lidar.h"
#include "modules/localization/msf/local_integ/localization_params.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/localization/proto/measure.pb.h"
#include "modules/localization/proto/sins_pva.pb.h"
#include "include/pose_forcast.h"
#include "include/sins_struct.h"

/**
 * @namespace apollo::localization
 * @brief apollo::localization
 */
namespace apollo {
namespace localization {
namespace msf {

enum class ForcastState {
    NOT_VALID = 0,
    INITIAL,
    INCREMENT
};

enum class LidarState {
  NOT_VALID = 0,
  NOT_STABLE,
  OK
};

enum class PredictLocationState {
  NOT_VALID = 0,
  INSPVA_ONLY,
  INSPVA_IMU,
  INSPVA_IMU_WHEEL
};

enum class WheelspeedState {
  NOT_INIT = 0,
  TOO_NEW,
  NEED_WAIT,
  WAIT_TOO_LONG,
  HALF_OK,
  OK
};

enum class ImuState {
  NOT_INIT = 0,
  TOO_NEW,
  NEED_WAIT,
  WAIT_TOO_LONG,
  HALF_OK,
  OK
};

enum class INSPVAState { NOT_INIT = 0, TOO_NEW, WAIT_TOO_LONG, OK };

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
  apollo::common::Status Init(const LocalizationIntegParam& params);
  // Lidar pcd process and get result.
  void PcdProcess(const LidarFrame& lidar_frame);
  void GetResult(int *lidar_status, TransformD *location, Matrix3D *covariance);
  int GetResult(LocalizationEstimate *lidar_local_msg);
  // Integrated navagation pva process.
  void IntegPvaProcess(const InsPva& sins_pva_msg);
  // Raw Imu process.
  void RawImuProcess(const ImuData& imu_msg);

 private:
  // Sub-functions for process.
  bool GetPredictPose(double lidar_time,
                      TransformD *inspva_pose,
                      ForcastState *forcast_state);
  bool CheckState();
  bool CheckDelta(const LidarFrame& frame, const TransformD& inspva_pose);
  void UpdateState(int ret, double time);

  // Load lidar-imu extrinsic parameter.
  bool LoadLidarExtrinsic(const std::string& file_path,
                          TransformD *lidar_extrinsic);
  // Load lidar height (the distance between lidar and ground).
  bool LoadLidarHeight(const std::string& file_path, LidarHeight *height);

  double ComputeDeltaYawLimit(int64_t index_cur, int64_t index_stable,
            double limit_min, double limit_max);

 private:
  // Lidar localization.
  LocalizationLidar *locator_;
  PoseForcast *pose_forcastor_;

  std::string map_path_;
  std::string lidar_extrinsic_file_;
  std::string lidar_height_file_;
  bool debug_log_flag_;
  int localization_mode_;
  int yaw_align_mode_;
  int lidar_filter_size_;
  double delta_yaw_limit_;
  double init_delta_yaw_limit_;
  double compensate_pitch_roll_limit_;
  int utm_zone_id_;
  double map_coverage_theshold_;
  TransformD lidar_extrinsic_;
  LidarHeight lidar_height_;

  bool is_pre_state_init_;
  TransformD cur_predict_location_;
  TransformD pre_predict_location_;
  Vector3D velocity_;
  TransformD pre_location_;
  TransformD location_;
  double pre_location_time_;

  // Information used to output.
  Matrix3D location_covariance_;
  LidarState lidar_status_;

  bool reinit_flag_;

  // imu and lidar max delay time
  double imu_lidar_max_delay_time_;

  int non_zero_odometry_cnt_;
  int max_nan_zero_odemetry_;

  WheelspeedState wheelspeed_state_;
  ImuState imu_state_;
  INSPVAState inspva_state_;

  int out_map_count_;

  int unstable_count_;
  double unstable_threshold_;
  bool is_unstable_reset_;

  /**@brief forcast integ pose, use to limit output of yaw */
  ForcastState forcast_integ_state_;
  int64_t forcast_timer_;
};

}  // namespace msf
}  // namespace localization
}  // namespace apollo

#endif  // MODULES_LOCALIZATION_MSF_LOCALIZATION_LIDAR_PROCESS_H_
