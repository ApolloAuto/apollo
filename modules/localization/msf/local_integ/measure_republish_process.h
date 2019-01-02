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
 * @file measure_republish_process.h
 * @brief The class of MeasureRepublishProcess
 */

#pragma once

#include <list>
#include <mutex>
#include <string>

#include "Eigen/Core"
#include "Eigen/Geometry"

#include "include/sins_struct.h"
#include "modules/common/status/status.h"
#include "modules/drivers/gnss/proto/gnss_best_pose.pb.h"
#include "modules/drivers/gnss/proto/heading.pb.h"
#include "modules/localization/msf/common/util/frame_transform.h"
#include "modules/localization/msf/local_integ/localization_params.h"
#include "modules/localization/proto/localization.pb.h"

/**
 * @namespace apollo::localization::msf
 * @brief apollo::localization::msf
 */
namespace apollo {
namespace localization {
namespace msf {

enum class GnssMode { NOVATEL = 0, SELF };

struct VehicleGnssAntExtrinsic {
  int ant_num;
  TransformD transform_1;
  TransformD transform_2;
};

/**
 * @class MeasureRepublishProcess
 *
 * @brief process lidar msg for localization
 */
class MeasureRepublishProcess {
 public:
  typedef drivers::gnss::GnssBestPose GnssBestPose;

  MeasureRepublishProcess();
  ~MeasureRepublishProcess();
  // Initialization.
  common::Status Init(const LocalizationIntegParam& params);

  // GNSS message process
  bool NovatelBestgnssposProcess(const GnssBestPose& bestgnsspos_msg,
                                 MeasureData* measure);
  void GnssLocalProcess(const MeasureData& gnss_local_msg,
                        MeasureData* measure);

  // integrated message process
  void IntegPvaProcess(const InsPva& inspva_msg);

  // lidar message process
  bool LidarLocalProcess(const LocalizationEstimate& lidar_local_msg,
                         MeasureData* measure);

  bool GnssHeadingProcess(const drivers::gnss::Heading& heading_msg,
                          MeasureData* measure, int* status);

 protected:
  bool IsSinsAlign();
  bool CheckBestgnssposeStatus(const GnssBestPose& bestgnsspos_msg);
  bool CheckBestgnssPoseXYStd(const GnssBestPose& bestgnsspos_msg);
  void TransferXYZFromBestgnsspose(const GnssBestPose& bestgnsspos_msg,
                                   MeasureData* measure);
  void TransferFirstMeasureFromBestgnsspose(const GnssBestPose& bestgnsspos_msg,
                                            MeasureData* measure);
  bool CalculateVelFromBestgnsspose(const GnssBestPose& bestgnsspos_msg,
                                    MeasureData* measure);

  bool LoadImuGnssAntennaExtrinsic(std::string file_path,
                                   VehicleGnssAntExtrinsic* extrinsic) const;

 private:
  MeasureData pre_bestgnsspose_;
  bool pre_bestgnsspose_valid_;
  bool send_init_bestgnsspose_;

  std::list<InsPva> integ_pva_list_;
  size_t pva_buffer_size_;
  std::mutex integ_pva_mutex_;

  int local_utm_zone_id_;
  bool is_trans_gpstime_to_utctime_;

  double map_height_time_;
  std::mutex height_mutex_;

  bool is_using_novatel_heading_;
  double novatel_heading_time_;
  std::mutex novatel_heading_mutex_;

  GnssMode gnss_mode_;

  static constexpr double GNSS_XY_STD_THRESHOLD = 5.0;
  static constexpr double BESTPOSE_TIME_MAX_INTERVAL = 1.05;
  static constexpr int BESTPOSE_GOOD_COUNT = 10;
  VehicleGnssAntExtrinsic imu_gnssant_extrinsic_;
};

}  // namespace msf
}  // namespace localization
}  // namespace apollo
