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

#include "modules/localization/msf/local_integ/gnss_msg_transfer.h"
#include "modules/common/log.h"

namespace apollo {
namespace localization {
namespace msf {

void GnssMagTransfer::Transfer(
    const apollo::drivers::gnss::BandObservation &in,
    BandObservationMsg* out) {
  CHECK_NOTNULL(out);
  if (in.has_band_id()) {
    out->set_band_id(GnssBandID(in.band_id()));
  }
  if (in.has_frequency_value()) {
    out->set_frequency_value(in.frequency_value());
  }
  if (in.has_pseudo_type()) {
    out->set_pseudo_type(PseudoType(in.pseudo_type()));
  }
  if (in.has_pseudo_range()) {
    out->set_pseudo_range(in.pseudo_range());
  }
  if (in.has_carrier_phase()) {
    out->set_carrier_phase(in.carrier_phase());
  }
  if (in.has_loss_lock_index()) {
    out->set_loss_lock_index(in.loss_lock_index());
  }
  if (in.has_doppler()) {
    out->set_doppler(in.doppler());
  }
  if (in.has_snr()) {
    out->set_snr(in.snr());
  }
  return;
}

void GnssMagTransfer::Transfer(
    const apollo::drivers::gnss::SatelliteObservation &in,
    SatelliteObservationMsg* out) {
  CHECK_NOTNULL(out);
  if (in.has_sat_prn()) {
    out->set_sat_prn(in.sat_prn());
  }
  if (in.has_sat_sys()) {
    out->set_sat_sys(GnssType(in.sat_sys()));
  }
  if (in.has_band_obs_num()) {
    out->set_band_obs_num(in.band_obs_num());
  }
  out->clear_band_obs();
  for (int idx = 0; idx < in.band_obs_size(); ++idx) {
    auto tmp = out->add_band_obs();
    Transfer(in.band_obs(idx), tmp);
  }

  return;
}

void GnssMagTransfer::Transfer(
    const apollo::drivers::gnss::EpochObservation &in,
    EpochObservationMsg* out) {
  CHECK_NOTNULL(out);
  if (in.has_receiver_id()) {
    out->set_receiver_id(in.receiver_id());
  }
  if (in.has_gnss_time_type()) {
    out->set_gnss_time_type(GnssTimeType(in.gnss_time_type()));
  }
  if (in.has_gnss_week()) {
    out->set_gnss_week(in.gnss_week());
  }
  if (in.has_gnss_second_s()) {
    out->set_gnss_second_s(in.gnss_second_s());
  }
  if (in.has_position_x()) {
    out->set_position_x(in.position_x());
  }
  if (in.has_position_y()) {
    out->set_position_y(in.position_y());
  }
  if (in.has_position_z()) {
    out->set_position_z(in.position_z());
  }
  if (in.has_health_flag()) {
    out->set_health_flag(in.health_flag());
  }
  if (in.has_sat_obs_num()) {
    out->set_sat_obs_num(in.sat_obs_num());
  }
  out->clear_sat_obs();
  for (int idx = 0; idx < in.sat_obs_size(); ++idx) {
    auto tmp = out->add_sat_obs();
    Transfer(in.sat_obs(idx), tmp);
  }
  return;
}

void GnssMagTransfer::Transfer(
    const apollo::drivers::gnss::KepplerOrbit &in,
    KepplerOrbitMsg* out) {
  CHECK_NOTNULL(out);
  if (in.has_gnss_type()) {
    out->set_gnss_type(GnssType(in.gnss_type()));
  }
  if (in.has_sat_prn()) {
    out->set_sat_prn(in.sat_prn());
  }
  if (in.has_gnss_time_type()) {
    out->set_gnss_time_type(GnssTimeType(in.gnss_time_type()));
  }
  if (in.has_year()) {
    out->set_year(in.year());
  }
  if (in.has_month()) {
    out->set_month(in.month());
  }
  if (in.has_day()) {
    out->set_day(in.day());
  }
  if (in.has_hour()) {
    out->set_hour(in.hour());
  }
  if (in.has_minute()) {
    out->set_minute(in.minute());
  }
  if (in.has_second_s()) {
    out->set_second_s(in.second_s());
  }
  if (in.has_week_num()) {
    out->set_week_num(in.week_num());
  }
  if (in.has_reserved()) {
    out->set_reserved(in.reserved());
  }
  if (in.has_af0()) {
    out->set_af0(in.af0());
  }
  if (in.has_af1()) {
    out->set_af1(in.af1());
  }
  if (in.has_af2()) {
    out->set_af2(in.af2());
  }
  if (in.has_iode()) {
    out->set_iode(in.iode());
  }
  if (in.has_deltan()) {
    out->set_deltan(in.deltan());
  }
  if (in.has_m0()) {
    out->set_m0(in.m0());
  }
  if (in.has_e()) {
    out->set_e(in.e());
  }
  if (in.has_roota()) {
    out->set_roota(in.roota());
  }
  if (in.has_toe()) {
    out->set_toe(in.toe());
  }
  if (in.has_toc()) {
    out->set_toc(in.toc());
  }
  if (in.has_cic()) {
    out->set_cic(in.cic());
  }
  if (in.has_crc()) {
    out->set_crc(in.crc());
  }
  if (in.has_cis()) {
    out->set_cis(in.cis());
  }
  if (in.has_crs()) {
    out->set_crs(in.crs());
  }
  if (in.has_cuc()) {
    out->set_cuc(in.cuc());
  }
  if (in.has_cus()) {
    out->set_cus(in.cus());
  }
  if (in.has_omega0()) {
    out->set_omega0(in.omega0());
  }
  if (in.has_omega()) {
    out->set_omega(in.omega());
  }
  if (in.has_i0()) {
    out->set_i0(in.i0());
  }
  if (in.has_omegadot()) {
    out->set_omegadot(in.omegadot());
  }
  if (in.has_idot()) {
    out->set_idot(in.idot());
  }
  if (in.has_codesonl2channel()) {
    out->set_codesonl2channel(in.codesonl2channel());
  }
  if (in.has_l2pdataflag()) {
    out->set_l2pdataflag(in.l2pdataflag());
  }
  if (in.has_accuracy()) {
    out->set_accuracy(in.accuracy());
  }
  if (in.has_health()) {
    out->set_health(in.health());
  }
  if (in.has_tgd()) {
    out->set_tgd(in.tgd());
  }
  if (in.has_iodc()) {
    out->set_iodc(in.iodc());
  }

  return;
}

void GnssMagTransfer::Transfer(
    const apollo::drivers::gnss::GlonassOrbit &in,
    GlonassOrbitMsg* out) {
  CHECK_NOTNULL(out);
  if (in.has_gnss_type()) {
    out->set_gnss_type(GnssType(in.gnss_type()));
  }
  if (in.has_slot_prn()) {
    out->set_slot_prn(in.slot_prn());
  }
  if (in.has_gnss_time_type()) {
    out->set_gnss_time_type(GnssTimeType(in.gnss_time_type()));
  }
  if (in.has_toe()) {
    out->set_toe(in.toe());
  }
  if (in.has_year()) {
    out->set_year(in.year());
  }
  if (in.has_month()) {
    out->set_month(in.month());
  }
  if (in.has_day()) {
    out->set_day(in.day());
  }
  if (in.has_hour()) {
    out->set_hour(in.hour());
  }
  if (in.has_minute()) {
    out->set_minute(in.minute());
  }
  if (in.has_second_s()) {
    out->set_second_s(in.second_s());
  }
  if (in.has_frequency_no()) {
    out->set_frequency_no(in.frequency_no());
  }
  if (in.has_week_num()) {
    out->set_week_num(in.week_num());
  }
  if (in.has_week_second_s()) {
    out->set_week_second_s(in.week_second_s());
  }
  if (in.has_tk()) {
    out->set_tk(in.tk());
  }
  if (in.has_clock_offset()) {
    out->set_clock_offset(in.clock_offset());
  }
  if (in.has_clock_drift()) {
    out->set_clock_drift(in.clock_drift());
  }
  if (in.has_health()) {
    out->set_health(in.health());
  }
  if (in.has_position_x()) {
    out->set_position_x(in.position_x());
  }
  if (in.has_position_y()) {
    out->set_position_y(in.position_y());
  }
  if (in.has_position_z()) {
    out->set_position_z(in.position_z());
  }
  if (in.has_velocity_x()) {
    out->set_velocity_x(in.velocity_x());
  }
  if (in.has_velocity_y()) {
    out->set_velocity_y(in.velocity_y());
  }
  if (in.has_velocity_z()) {
    out->set_velocity_z(in.velocity_z());
  }
  if (in.has_accelerate_x()) {
    out->set_accelerate_x(in.accelerate_x());
  }
  if (in.has_accelerate_y()) {
    out->set_accelerate_y(in.accelerate_y());
  }
  if (in.has_accelerate_z()) {
    out->set_accelerate_z(in.accelerate_z());
  }
  if (in.has_infor_age()) {
    out->set_infor_age(in.infor_age());
  }

  return;
}

void GnssMagTransfer::Transfer(
    const apollo::drivers::gnss::GnssEphemeris &in,
    GnssEphemerisMsg* out) {
  CHECK_NOTNULL(out);
  if (in.has_gnss_type()) {
    out->set_gnss_type(GnssType(in.gnss_type()));
  }
  if (in.has_keppler_orbit()) {
    auto tmp = out->mutable_keppler_orbit();
    Transfer(in.keppler_orbit(), tmp);
  }
  if (in.has_glonass_orbit()) {
    auto tmp = out->mutable_glonass_orbit();
    Transfer(in.glonass_orbit(), tmp);
  }

  return;
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
