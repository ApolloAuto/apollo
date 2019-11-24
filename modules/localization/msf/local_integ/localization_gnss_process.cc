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

#include "modules/localization/msf/local_integ/localization_gnss_process.h"

#include "yaml-cpp/yaml.h"

#include "cyber/common/log.h"
#include "modules/common/time/time.h"
#include "modules/localization/msf/local_integ/gnss_msg_transfer.h"

namespace apollo {
namespace localization {
namespace msf {

using apollo::common::Status;

LocalizationGnssProcess::LocalizationGnssProcess()
    : gnss_solver_(new GnssSolver()),
      enable_ins_aid_rtk_(true),
      gnss_lever_arm_{0.0, 0.0, 0.0},
      sins_align_finish_(false),
      double_antenna_solver_(new GnssSolver()),
      current_obs_time_(0.0),
      gnss_state_(LocalizationMeasureState::NOT_VALID) {
  SetDefaultOption();
}

LocalizationGnssProcess::~LocalizationGnssProcess() {
  map_gnss_eph_.clear();

  delete gnss_solver_;
  gnss_solver_ = nullptr;
  delete double_antenna_solver_;
  double_antenna_solver_ = nullptr;
}

Status LocalizationGnssProcess::Init(const LocalizationIntegParam &param) {
  // set launch parameter
  enable_ins_aid_rtk_ = param.enable_ins_aid_rtk;
  gnss_solver_->set_enable_external_prediction(enable_ins_aid_rtk_);

  gnss_lever_arm_.arm_x = param.imu_to_ant_offset.offset_x;
  gnss_lever_arm_.arm_y = param.imu_to_ant_offset.offset_y;
  gnss_lever_arm_.arm_z = param.imu_to_ant_offset.offset_z;

  map_gnss_eph_.clear();
  sins_align_finish_ = false;

  return Status::OK();
}

void LocalizationGnssProcess::SetDefaultOption() {
  // set default process modes
  gnss_solver_->set_position_option(3);
  gnss_solver_->set_tropsphere_option(0);
  gnss_solver_->enable_cycle_slip_fix();
  enable_ins_aid_rtk_ = false;

  gnss_lever_arm_.arm_x = -0.030;
  gnss_lever_arm_.arm_y = 0.338;
  gnss_lever_arm_.arm_z = 1.291;
}

void LocalizationGnssProcess::RawObservationProcess(
    const drivers::gnss::EpochObservation &raw_obs) {
  if (!raw_obs.has_receiver_id()) {
    AERROR << "Obs data being invalid if without receiver id!";
    return;
  }
  const double unix_to_gps = 315964800;
  const double sec_per_week = 604800;
  double leap_second_s = gnss_solver_->get_leap_second(raw_obs.gnss_week(),
                                                       raw_obs.gnss_second_s());

  double sys_secs_to_gnss =
      common::time::Clock::NowInSeconds() - unix_to_gps + leap_second_s;
  double obs_secs =
      raw_obs.gnss_second_s() + raw_obs.gnss_week() * sec_per_week;
  double obs_delay = sys_secs_to_gnss - obs_secs;

  double obs_xyz[3] = {0.0};
  if (raw_obs.has_position_x() && raw_obs.has_position_y() &&
      raw_obs.has_position_z()) {
    obs_xyz[0] = raw_obs.position_x();
    obs_xyz[1] = raw_obs.position_y();
    obs_xyz[2] = raw_obs.position_z();
  }
  const unsigned int max_msg_size = 256;
  char message[max_msg_size] = {'\0'};
  snprintf(
      message, max_msg_size,
      "user %u time:%12.3f sat_num:%d obs_delay:%12.3f%16.3f%16.3f%16.3f\n",
      raw_obs.receiver_id(), raw_obs.gnss_second_s(), raw_obs.sat_obs_num(),
      obs_delay, obs_xyz[0], obs_xyz[1], obs_xyz[2]);
  AINFO << message;

  EpochObservationMsg raw_obs_msg;
  GnssMagTransfer::Transfer(raw_obs, &raw_obs_msg);
  if (raw_obs.receiver_id() != 0) {
    // Notice: the baser coordinate should be binded with obs together anytime.
    gnss_solver_->save_baser_observation(raw_obs_msg);
    // id - 0x80000000 = station_id from gnss_driver
    return;
  }

  const unsigned int second_per_week = 604800;
  double new_obs_time =
      raw_obs.gnss_second_s() + raw_obs.gnss_week() * second_per_week;
  if (new_obs_time <= current_obs_time_) {
    return;
  }
  current_obs_time_ = new_obs_time;
  GnssPosition(&raw_obs_msg);
}

void LocalizationGnssProcess::RawEphemerisProcess(
    const drivers::gnss::GnssEphemeris &msg) {
  if (!msg.has_gnss_type()) {
    return;
  }
  auto gnss_orbit = msg;
  if (gnss_orbit.gnss_type() == drivers::gnss::GnssType::GLO_SYS) {
    /* caros driver (derived from rtklib src) set glonass eph toe as the GPST,
     * and here convert it back to UTC(+0), so leap seconds should be in
     * accordance with the GNSS-Driver
     */
    double leap_sec =
        gnss_solver_->get_leap_second(gnss_orbit.glonass_orbit().week_num(),
                                      gnss_orbit.glonass_orbit().toe());
    double toe = gnss_orbit.glonass_orbit().toe() - leap_sec;
    gnss_orbit.mutable_glonass_orbit()->set_toe(toe);
    gnss_orbit.mutable_glonass_orbit()->set_week_second_s(toe);
  }
  static int eph_counter = 0;
  ++eph_counter;
  // printf("received a gnss ephemeris: %d!\n", eph_counter);
  if (DuplicateEph(gnss_orbit)) {
    AINFO << "received an existed gnss ephemeris!";
    return;
  }

  GnssEphemerisMsg gnss_orbit_msg;
  GnssMagTransfer::Transfer(gnss_orbit, &gnss_orbit_msg);
  gnss_solver_->save_gnss_ephemris(gnss_orbit_msg);
}

void LocalizationGnssProcess::IntegSinsPvaProcess(const InsPva &sins_pva_msg,
                                                  const double variance[9][9]) {
  if (!sins_pva_msg.init_and_alignment) {
    return;
  }

  sins_align_finish_ = true;

  // feed into GNSS-RTK Engine
  double sec_s = sins_pva_msg.time;
  double llh[3] = {sins_pva_msg.pos.longitude, sins_pva_msg.pos.latitude,
                   sins_pva_msg.pos.height};
  double velocity[3] = {sins_pva_msg.vel.ve, sins_pva_msg.vel.vn,
                        sins_pva_msg.vel.vu};
  double lever_arm[3] = {gnss_lever_arm_.arm_x, gnss_lever_arm_.arm_y,
                         gnss_lever_arm_.arm_z};
  double euler[3] = {sins_pva_msg.att.pitch, sins_pva_msg.att.roll,
                     sins_pva_msg.att.yaw};

  double std_pos[3][3] = {0.0};
  double std_vel[3][3] = {0.0};
  // const unsigned int dim = 9;
  for (unsigned int i = 0; i < 3; ++i) {
    for (unsigned int j = 0; j < 3; ++j) {
      std_pos[i][j] = variance[i][j];
      std_vel[i][j] = variance[i + 3][j + 3];
    }
  }

  gnss_solver_->motion_update(sec_s, llh, std_pos, velocity, std_vel, euler,
                              lever_arm);
}

LocalizationMeasureState LocalizationGnssProcess::GetResult(
    MeasureData *gnss_msg) {
  CHECK_NOTNULL(gnss_msg);

  // convert GnssPntResult to IntegMeasure
  // double sec_s = Clock::NowInSeconds(); // ros::Time::now().toSec();
  const unsigned int second_per_week = 604800;
  double sec_s = gnss_pnt_result_.gnss_week() * second_per_week +
                 gnss_pnt_result_.gnss_second_s();
  gnss_msg->time = sec_s;
  gnss_msg->frame_type = FrameType::ECEF;

  // velocity must be in ENU frame and position in frame WGS84
  const int dim = 9;
  for (int i = 0; i < dim; ++i) {
    for (int j = 0; j < dim; ++j) {
      gnss_msg->variance[i][j] = 0.0;
    }
  }

  bool b_pos = false;
  if (gnss_pnt_result_.has_pos_x_m() && gnss_pnt_result_.has_pos_y_m() &&
      gnss_pnt_result_.has_pos_z_m()) {
    b_pos = true;
    gnss_msg->gnss_pos.longitude = gnss_pnt_result_.pos_x_m();
    gnss_msg->gnss_pos.latitude = gnss_pnt_result_.pos_y_m();
    gnss_msg->gnss_pos.height = gnss_pnt_result_.pos_z_m();
    gnss_msg->measure_type = MeasureType::GNSS_POS_ONLY;

    gnss_msg->is_have_variance = true;
    // std
    gnss_msg->variance[0][0] =
        gnss_pnt_result_.std_pos_x_m() * gnss_pnt_result_.std_pos_x_m();
    gnss_msg->variance[1][1] =
        gnss_pnt_result_.std_pos_y_m() * gnss_pnt_result_.std_pos_y_m();
    gnss_msg->variance[2][2] =
        gnss_pnt_result_.std_pos_z_m() * gnss_pnt_result_.std_pos_z_m();
  }
  bool b_vel = false;
  if (gnss_pnt_result_.has_vel_x_m() && gnss_pnt_result_.has_vel_y_m() &&
      gnss_pnt_result_.has_vel_z_m()) {
    // output velocity whenever possible.
    b_vel = true;
    gnss_msg->gnss_vel.ve = gnss_pnt_result_.vel_x_m();
    gnss_msg->gnss_vel.vn = gnss_pnt_result_.vel_y_m();
    gnss_msg->gnss_vel.vu = gnss_pnt_result_.vel_z_m();
    gnss_msg->measure_type = MeasureType::ENU_VEL_ONLY;
    gnss_msg->is_have_variance = true;
    // std
    gnss_msg->variance[3][3] =
        gnss_pnt_result_.std_vel_x_m() * gnss_pnt_result_.std_vel_x_m();
    gnss_msg->variance[4][4] =
        gnss_pnt_result_.std_vel_y_m() * gnss_pnt_result_.std_vel_y_m();
    gnss_msg->variance[5][5] =
        gnss_pnt_result_.std_vel_z_m() * gnss_pnt_result_.std_vel_z_m();
  }
  if (b_pos && b_vel) {
    gnss_msg->measure_type = MeasureType::GNSS_POS_VEL;
  }

  return gnss_state_;
}

bool LocalizationGnssProcess::DuplicateEph(
    const drivers::gnss::GnssEphemeris &raw_eph) {
  drivers::gnss::GnssType t_type = raw_eph.gnss_type();
  if (t_type != drivers::gnss::GnssType::GPS_SYS &&
      t_type != drivers::gnss::GnssType::BDS_SYS &&
      t_type != drivers::gnss::GnssType::GLO_SYS) {
    return true;
  }
  unsigned int sat_prn = 0;
  unsigned int week_num = 0;
  double toe = -0.1;
  if (t_type == drivers::gnss::GnssType::GLO_SYS) {
    sat_prn = raw_eph.glonass_orbit().slot_prn();
    week_num = raw_eph.glonass_orbit().week_num();
    toe = raw_eph.glonass_orbit().toe();
  } else {
    sat_prn = raw_eph.keppler_orbit().sat_prn();
    week_num = raw_eph.keppler_orbit().week_num();
    toe = raw_eph.keppler_orbit().toe();
  }
  const EphKey temp(drivers::gnss::GnssType(t_type), sat_prn, week_num, toe);
  if (map_gnss_eph_.find(temp) != map_gnss_eph_.end()) {
    return true;
  }
  map_gnss_eph_.insert(
      std::map<EphKey, drivers::gnss::GnssEphemeris>::value_type(temp,
                                                                 raw_eph));
  return false;
}

inline void LocalizationGnssProcess::LogPnt(const GnssPntResultMsg &rover_pnt,
                                            double ratio) {
  char print_infor[256] = {'\0'};
  snprintf(print_infor, sizeof(print_infor),
           "%6d%12.3f%4d%16.3f%16.3f%16.3f%4d%4.1f%6.1f%8.3f%8.3f%8.3f%8.3f%8."
           "3f%8.3f\n",
           rover_pnt.gnss_week(), rover_pnt.gnss_second_s(),
           static_cast<int>(rover_pnt.pnt_type()), rover_pnt.pos_x_m(),
           rover_pnt.pos_y_m(), rover_pnt.pos_z_m(), rover_pnt.sovled_sat_num(),
           rover_pnt.pdop(), ratio, rover_pnt.vel_x_m(), rover_pnt.vel_y_m(),
           rover_pnt.vel_z_m(), rover_pnt.std_pos_x_m(),
           rover_pnt.std_pos_y_m(), rover_pnt.std_pos_z_m());
  AINFO << print_infor;
}

bool LocalizationGnssProcess::GnssPosition(EpochObservationMsg *raw_rover_obs) {
  CHECK_NOTNULL(raw_rover_obs);

  gnss_state_ = LocalizationMeasureState::NOT_VALID;
  if (raw_rover_obs->receiver_id() != 0) {
    AERROR << "Wrong Rover Obs Data!";
    return false;
  }
  int b_solved = gnss_solver_->solve(raw_rover_obs, &gnss_pnt_result_);
  if (b_solved < 0) {
    return false;
  }
  LogPnt(gnss_pnt_result_, gnss_solver_->get_ratio());
  if (!sins_align_finish_) {
    AWARN << "Sins-ekf has not converged or finished its alignment!";
  }
  if (gnss_pnt_result_.has_std_pos_x_m() &&
      gnss_pnt_result_.has_std_pos_y_m() &&
      gnss_pnt_result_.has_std_pos_z_m()) {
    double sigma =
        gnss_pnt_result_.std_pos_x_m() * gnss_pnt_result_.std_pos_x_m() +
        gnss_pnt_result_.std_pos_y_m() * gnss_pnt_result_.std_pos_y_m() +
        gnss_pnt_result_.std_pos_z_m() * gnss_pnt_result_.std_pos_z_m();
    sigma = std::sqrt(fabs(sigma));
    const double sigma_threshold = 10.0;
    if (fabs(sigma) > sigma_threshold) {
      AWARN << "Position std exceeds the threshold " << sigma_threshold << "!";
      return false;
    }
  }

  gnss_state_ = LocalizationMeasureState::OK;

  return true;
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
