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

#pragma once

#include "gnss_struct.h"
#include "sins_struct.h"

namespace apollo {
namespace localization {
namespace msf {

class GnssPntSolver;
class GnssSolver {
 public:
  GnssSolver();
  ~GnssSolver();

 //set before solve
  bool set_position_option(int option);
  int get_position_option();

  void set_enable_external_prediction(bool b_enable);
  bool get_enable_external_prediction();

  bool set_tropsphere_option(int option);
  bool set_ionosphere_option(int option);

  bool set_rtk_result_file(char* rtk_result_file);
  bool set_ambiguity_file(char* ambiguity_recorder_file);

  void enable_half_cycle_ar(const bool b_enable);
  bool get_enable_half_cycle_ar();
  void enable_cycle_slip_fix();

  //ephemeris
  bool save_gnss_ephemris(const GnssEphemerisMsg& gnss_orbit);

  //observation
  bool save_baser_observation(const EpochObservationMsg& baser_obs);

  int solve(EpochObservationMsg *rover_obs,
       apollo::localization::msf::GnssPntResultMsg *rover_pnt);

  bool motion_update_xyz(double time_sec,
                         const double position[3],
                         const double std_pos[3][3],
                         const double velocity[3],
                         const double std_vel[3][3]);
  bool motion_update(double time_sec,
                     const double position[3],
                     const double std_pos[3][3],
                     const double velocity[3],
                     const double std_vel[3][3],
                     const double euler[3],
                     const double lever_arm[3]);

  double get_leap_second(unsigned int gps_week_num,
                         double gps_week_second_s);
  double get_ratio();

 private:
  GnssPntSolver *solver_;
};

}  // namespace msf
}  // namespace localization
}  // namespace apollo