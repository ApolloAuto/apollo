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
 * @file localization_gnss_process.h
 * @brief The class of LocalizationGnssProcess
 */

#ifndef MODULES_LOCALIZATION_MSF_LOCALIZATION_GNSS_PROCESS_H_
#define MODULES_LOCALIZATION_MSF_LOCALIZATION_GNSS_PROCESS_H_

#include <map>
#include <string>

#include "Eigen/Core"
#include "Eigen/Geometry"

#include "modules/common/status/status.h"
#include "modules/drivers/gnss/proto/gnss_raw_observation.pb.h"
#include "modules/localization/msf/local_integ/localization_params.h"
#include "include/gnss_solver.h"

/**
 * @namespace apollo::localization::msf
 * @brief apollo::localization::msf
 */
namespace apollo {
namespace localization {
namespace msf {

union LeverArm {
  double arm[3];
  struct {
    double arm_x;
    double arm_y;
    double arm_z;
  };
};

struct EphKey {
  apollo::drivers::gnss::GnssType gnss_type;
  unsigned int sat_prn;
  // toe = eph.toe + eph.week_num * sec_per_week
  double eph_toe;
  static const int second_per_week = 604800;
  EphKey(const apollo::drivers::gnss::GnssType type,
         const unsigned int prn,
         double toe) {
    gnss_type = type;
    sat_prn = prn;
    eph_toe = toe;
  }
  EphKey(const apollo::drivers::gnss::GnssType type,
         const unsigned int prn,
         const unsigned int week_num,
         double toe) {
    gnss_type = type;
    sat_prn = prn;
    eph_toe = toe + week_num * second_per_week;
  }
  EphKey() {
    gnss_type = apollo::drivers::gnss::SYS_UNKNOWN;
    sat_prn = 0;
    eph_toe = -0.1;
  }
  bool operator < (const EphKey& key2) const {
    if (gnss_type < key2.gnss_type) {
      return true;
    }
    if (gnss_type == key2.gnss_type) {
      if (sat_prn < key2.sat_prn) {
        return true;
      }
      if (sat_prn == key2.sat_prn) {
        return eph_toe < key2.eph_toe;
      }
      return false;
    }
    return false;
  }
  bool operator == (const EphKey& key2) const {
    return (gnss_type == key2.gnss_type)
           && (sat_prn == key2.sat_prn)
           && (eph_toe == key2.eph_toe);
  }
  EphKey& operator = (const EphKey& key2) {
    gnss_type = key2.gnss_type;
    sat_prn = key2.sat_prn;
    eph_toe = key2.eph_toe;
    return *this;
  }
};

class LocalizationGnssProcess {
 public:
  LocalizationGnssProcess();
  ~LocalizationGnssProcess();
  apollo::common::Status Init(const LocalizationIntegParam &param);
  // callback function for rostopic
  // raw data' field "receiver_id" differs rover (= 0) from baser (= 1)
  void RawObservationProcess(const drivers::gnss::EpochObservation &raw_obs);
  void RawEphemerisProcess(const drivers::gnss::GnssEphemeris &gnss_orbit);
  void IntegSinsPvaProcess(const InsPva &sins_pva,
                           const double variance[9][9]);
  LocalizationMeasureState GetResult(MeasureData *gnss_measure);

 private:
  void SetDefaultOption();
  // bool LoadHistoryEph(const std::string &nav_file);
  bool DuplicateEph(const drivers::gnss::GnssEphemeris &raw_eph);

  inline void LogPnt(const GnssPntResultMsg &rover_pnt, double ratio);
  bool GnssPosition(EpochObservationMsg *raw_rover_obs);

 private:
  GnssSolver *gnss_solver_;
  GnssPntResultMsg gnss_pnt_result_;

  bool enable_ins_aid_rtk_;

  std::map<EphKey, drivers::gnss::GnssEphemeris> map_gnss_eph_;

  // from imu to gnss antenna
  LeverArm gnss_lever_arm_;
  // integrated-ins indicator
  bool sins_align_finish_;

  // deploy simple short-baseline to resolve heading
  GnssSolver *double_antenna_solver_;

  // newest obs time
  double current_obs_time_;

  LocalizationMeasureState gnss_state_;
};

}  // namespace msf
}  // namespace localization
}  // namespace apollo

#endif  // MODULES_LOCALIZATION_MSF_LOCALIZATION_GNSS_PROCESS_H_
