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

typedef unsigned int uint32;
typedef int int32;

namespace apollo {
namespace localization {
namespace msf {

enum GnssBandID {
  BAND_UNKNOWN = 0,
  GPS_L1 = 1,
  GPS_L2 = 2,
  GPS_L5 = 3,
  BDS_B1 = 4,
  BDS_B2 = 5,
  BDS_B3 = 6,
  GLO_G1 = 7,
  GLO_G2 = 8,
  GLO_G3 = 9,
  GAL_E1 = 10,
  GAL_E5a = 11,
  GAL_E5b = 12,
  GAL_E6 = 13
};

enum GnssTimeType {
  TIME_UNKNOWN = 0,
  GPS_TIME = 1,
  BDS_TIME = 2,
  GLO_TIME = 3,
  GAL_TIME = 4
};

enum GnssType {
  SYS_UNKNOWN = 0,
  GPS_SYS = 1,
  BDS_SYS = 2,
  GLO_SYS = 3,
  GAL_SYS = 4
};

enum PseudoType {
  CODE_UNKNOWN = 0,
  CORSE_CODE = 1,
  PRECISION_CODE = 2
};

enum PntType {
  PNT_INVALID = 0,
  PNT_SPP = 1,
  PNT_PHASE_TD = 2,
  PNT_CODE_DIFF = 3,
  PNT_RTK_FLOAT = 4,
  PNT_RTK_FIXED = 5
};

class BandObservation;
class BandObservationMsg {
 public:
  BandObservationMsg();
  ~BandObservationMsg();

  void map(BandObservation *data);
  void unmap();
  BandObservation* mutable_data();
  const BandObservation& data() const;

  bool has_band_id() const;
  void clear_band_id();
  ::apollo::localization::msf::GnssBandID band_id() const;
  void set_band_id(::apollo::localization::msf::GnssBandID value);

  bool has_frequency_value() const;
  void clear_frequency_value();
  double frequency_value() const;
  void set_frequency_value(double value);

  bool has_pseudo_type() const;
  void clear_pseudo_type();
  ::apollo::localization::msf::PseudoType pseudo_type() const;
  void set_pseudo_type(::apollo::localization::msf::PseudoType value);

  bool has_pseudo_range() const;
  void clear_pseudo_range();
  double pseudo_range() const;
  void set_pseudo_range(double value);

  bool has_carrier_phase() const;
  void clear_carrier_phase();
  double carrier_phase() const;
  void set_carrier_phase(double value);

  bool has_loss_lock_index() const;
  void clear_loss_lock_index();
  uint32 loss_lock_index() const;
  void set_loss_lock_index(uint32 value);

  bool has_doppler() const;
  void clear_doppler();
  double doppler() const;
  void set_doppler(double value);

  bool has_snr() const;
  void clear_snr();
  float snr() const;
  void set_snr(float value);
 private:
  BandObservation *data_;
  bool is_mapped_;
};

class SatelliteObservation;
class SatelliteObservationMsg {
 public:
  SatelliteObservationMsg();
  ~SatelliteObservationMsg();

  void map(SatelliteObservation *data);
  void unmap();
  SatelliteObservation* mutable_data();
  const SatelliteObservation& data() const;

  bool has_sat_prn() const;
  void clear_sat_prn();
  uint32 sat_prn() const;
  void set_sat_prn(uint32 value);

  bool has_sat_sys() const;
  void clear_sat_sys();
  ::apollo::localization::msf::GnssType sat_sys() const;
  void set_sat_sys(::apollo::localization::msf::GnssType value);

  bool has_band_obs_num() const;
  void clear_band_obs_num();
  uint32 band_obs_num() const;
  void set_band_obs_num(uint32 value);

  int band_obs_size() const;
  void clear_band_obs();
  const ::apollo::localization::msf::BandObservationMsg& band_obs(int index);
  ::apollo::localization::msf::BandObservationMsg* mutable_band_obs(int index);
  ::apollo::localization::msf::BandObservationMsg* add_band_obs();
 private:
  SatelliteObservation *data_;
  bool is_mapped_;
  BandObservationMsg *band_obervation_msg_;
};

class EpochObservation;
class EpochObservationMsg {
 public:
  EpochObservationMsg();
  ~EpochObservationMsg();

  void map(EpochObservation *data);
  void unmap();
  EpochObservation* mutable_data();
  const EpochObservation& data() const;

  bool has_receiver_id() const;
  void clear_receiver_id();
  uint32 receiver_id() const;
  void set_receiver_id(uint32 value);

  bool has_gnss_time_type() const;
  void clear_gnss_time_type();
  ::apollo::localization::msf::GnssTimeType gnss_time_type() const;
  void set_gnss_time_type(::apollo::localization::msf::GnssTimeType value);

  bool has_gnss_week() const;
  void clear_gnss_week();
  uint32 gnss_week() const;
  void set_gnss_week(uint32 value);

  bool has_gnss_second_s() const;
  void clear_gnss_second_s();
  double gnss_second_s() const;
  void set_gnss_second_s(double value);

  bool has_position_x() const;
  void clear_position_x();
  double position_x() const;
  void set_position_x(double value);

  bool has_position_y() const;
  void clear_position_y();
  double position_y() const;
  void set_position_y(double value);

  bool has_position_z() const;
  void clear_position_z();
  double position_z() const;
  void set_position_z(double value);

  bool has_health_flag() const;
  void clear_health_flag();
  uint32 health_flag() const;
  void set_health_flag(uint32 value);

  bool has_sat_obs_num() const;
  void clear_sat_obs_num();
  uint32 sat_obs_num() const;
  void set_sat_obs_num(uint32 value);

  int sat_obs_size() const;
  void clear_sat_obs();
  const ::apollo::localization::msf::SatelliteObservationMsg& sat_obs(int index);
  ::apollo::localization::msf::SatelliteObservationMsg* mutable_sat_obs(int index);
  ::apollo::localization::msf::SatelliteObservationMsg* add_sat_obs();
 private:
  EpochObservation *data_;
  bool is_mapped_;
  SatelliteObservationMsg *satellite_observation_msg_;
};
// -------------------------------------------------------------------

class KepplerOrbit;
class KepplerOrbitMsg {
 public:
  KepplerOrbitMsg();
  // explicit KepplerOrbitMsg(KepplerOrbit *data);
  ~KepplerOrbitMsg();

  void map(KepplerOrbit *data);
  void unmap();
  KepplerOrbit* mutable_data();
  const KepplerOrbit& data() const;

  bool has_gnss_type() const;
  void clear_gnss_type();
  ::apollo::localization::msf::GnssType gnss_type() const;
  void set_gnss_type(::apollo::localization::msf::GnssType value);

  bool has_sat_prn() const;
  void clear_sat_prn();
  uint32 sat_prn() const;
  void set_sat_prn(uint32 value);

  bool has_gnss_time_type() const;
  void clear_gnss_time_type();
  ::apollo::localization::msf::GnssTimeType gnss_time_type() const;
  void set_gnss_time_type(::apollo::localization::msf::GnssTimeType value);

  bool has_year() const;
  void clear_year();
  uint32 year() const;
  void set_year(uint32 value);

  bool has_month() const;
  void clear_month();
  uint32 month() const;
  void set_month(uint32 value);

  bool has_day() const;
  void clear_day();
  uint32 day() const;
  void set_day(uint32 value);

  bool has_hour() const;
  void clear_hour();
  uint32 hour() const;
  void set_hour(uint32 value);

  bool has_minute() const;
  void clear_minute();
  uint32 minute() const;
  void set_minute(uint32 value);

  bool has_second_s() const;
  void clear_second_s();
  double second_s() const;
  void set_second_s(double value);

  bool has_week_num() const;
  void clear_week_num();
  uint32 week_num() const;
  void set_week_num(uint32 value);

  bool has_reserved() const;
  void clear_reserved();
  double reserved() const;
  void set_reserved(double value);

  bool has_af0() const;
  void clear_af0();
  double af0() const;
  void set_af0(double value);

  bool has_af1() const;
  void clear_af1();
  double af1() const;
  void set_af1(double value);

  bool has_af2() const;
  void clear_af2();
  double af2() const;
  void set_af2(double value);

  bool has_iode() const;
  void clear_iode();
  double iode() const;
  void set_iode(double value);

  bool has_deltan() const;
  void clear_deltan();
  double deltan() const;
  void set_deltan(double value);

  bool has_m0() const;
  void clear_m0();
  double m0() const;
  void set_m0(double value);

  bool has_e() const;
  void clear_e();
  double e() const;
  void set_e(double value);

  bool has_roota() const;
  void clear_roota();
  double roota() const;
  void set_roota(double value);

  bool has_toe() const;
  void clear_toe();
  double toe() const;
  void set_toe(double value);

  bool has_toc() const;
  void clear_toc();
  double toc() const;
  void set_toc(double value);

  bool has_cic() const;
  void clear_cic();
  double cic() const;
  void set_cic(double value);

  bool has_crc() const;
  void clear_crc();
  double crc() const;
  void set_crc(double value);

  bool has_cis() const;
  void clear_cis();
  double cis() const;
  void set_cis(double value);

  bool has_crs() const;
  void clear_crs();
  double crs() const;
  void set_crs(double value);

  bool has_cuc() const;
  void clear_cuc();
  double cuc() const;
  void set_cuc(double value);

  bool has_cus() const;
  void clear_cus();
  double cus() const;
  void set_cus(double value);

  bool has_omega0() const;
  void clear_omega0();
  double omega0() const;
  void set_omega0(double value);

  bool has_omega() const;
  void clear_omega();
  double omega() const;
  void set_omega(double value);

  bool has_i0() const;
  void clear_i0();
  double i0() const;
  void set_i0(double value);

  bool has_omegadot() const;
  void clear_omegadot();
  double omegadot() const;
  void set_omegadot(double value);

  bool has_idot() const;
  void clear_idot();
  double idot() const;
  void set_idot(double value);

  bool has_codesonl2channel() const;
  void clear_codesonl2channel();
  double codesonl2channel() const;
  void set_codesonl2channel(double value);

  bool has_l2pdataflag() const;
  void clear_l2pdataflag();
  uint32 l2pdataflag() const;
  void set_l2pdataflag(uint32 value);

  bool has_accuracy() const;
  void clear_accuracy();
  uint32 accuracy() const;
  void set_accuracy(uint32 value);

  bool has_health() const;
  void clear_health();
  uint32 health() const;
  void set_health(uint32 value);

  bool has_tgd() const;
  void clear_tgd();
  double tgd() const;
  void set_tgd(double value);

  bool has_iodc() const;
  void clear_iodc();
  double iodc() const;
  void set_iodc(double value);

 private:
  KepplerOrbit *data_;
  bool is_mapped_;
};
// -------------------------------------------------------------------

class GlonassOrbit;
class GlonassOrbitMsg {
 public:
  GlonassOrbitMsg();
  // explicit GlonassOrbitMsg(GlonassOrbit *data);
  ~GlonassOrbitMsg();

  void map(GlonassOrbit *data);
  void unmap();
  GlonassOrbit* mutable_data();
  const GlonassOrbit& data() const;

  bool has_gnss_type() const;
  void clear_gnss_type();
  ::apollo::localization::msf::GnssType gnss_type() const;
  void set_gnss_type(::apollo::localization::msf::GnssType value);

  bool has_slot_prn() const;
  void clear_slot_prn();
  uint32 slot_prn() const;
  void set_slot_prn(uint32 value);

  bool has_gnss_time_type() const;
  void clear_gnss_time_type();
  ::apollo::localization::msf::GnssTimeType gnss_time_type() const;
  void set_gnss_time_type(::apollo::localization::msf::GnssTimeType value);

  bool has_toe() const;
  void clear_toe();
  double toe() const;
  void set_toe(double value);

  bool has_year() const;
  void clear_year();
  uint32 year() const;
  void set_year(uint32 value);

  bool has_month() const;
  void clear_month();
  uint32 month() const;
  void set_month(uint32 value);

  bool has_day() const;
  void clear_day();
  uint32 day() const;
  void set_day(uint32 value);

  bool has_hour() const;
  void clear_hour();
  uint32 hour() const;
  void set_hour(uint32 value);

  bool has_minute() const;
  void clear_minute();
  uint32 minute() const;
  void set_minute(uint32 value);

  bool has_second_s() const;
  void clear_second_s();
  double second_s() const;
  void set_second_s(double value);

  bool has_frequency_no() const;
  void clear_frequency_no();
  int32 frequency_no() const;
  void set_frequency_no(int32 value);

  bool has_week_num() const;
  void clear_week_num();
  uint32 week_num() const;
  void set_week_num(uint32 value);

  bool has_week_second_s() const;
  void clear_week_second_s();
  double week_second_s() const;
  void set_week_second_s(double value);

  bool has_tk() const;
  void clear_tk();
  double tk() const;
  void set_tk(double value);

  bool has_clock_offset() const;
  void clear_clock_offset();
  double clock_offset() const;
  void set_clock_offset(double value);

  bool has_clock_drift() const;
  void clear_clock_drift();
  double clock_drift() const;
  void set_clock_drift(double value);

  bool has_health() const;
  void clear_health();
  uint32 health() const;
  void set_health(uint32 value);

  bool has_position_x() const;
  void clear_position_x();
  double position_x() const;
  void set_position_x(double value);

  bool has_position_y() const;
  void clear_position_y();
  double position_y() const;
  void set_position_y(double value);

  bool has_position_z() const;
  void clear_position_z();
  double position_z() const;
  void set_position_z(double value);

  bool has_velocity_x() const;
  void clear_velocity_x();
  double velocity_x() const;
  void set_velocity_x(double value);

  bool has_velocity_y() const;
  void clear_velocity_y();
  double velocity_y() const;
  void set_velocity_y(double value);

  bool has_velocity_z() const;
  void clear_velocity_z();
  double velocity_z() const;
  void set_velocity_z(double value);

  bool has_accelerate_x() const;
  void clear_accelerate_x();
  double accelerate_x() const;
  void set_accelerate_x(double value);

  bool has_accelerate_y() const;
  void clear_accelerate_y();
  double accelerate_y() const;
  void set_accelerate_y(double value);

  bool has_accelerate_z() const;
  void clear_accelerate_z();
  double accelerate_z() const;
  void set_accelerate_z(double value);

  bool has_infor_age() const;
  void clear_infor_age();
  double infor_age() const;
  void set_infor_age(double value);
 private:
  GlonassOrbit *data_;
  bool is_mapped_;
};

class GnssEphemeris;
class GnssEphemerisMsg {
 public:
  GnssEphemerisMsg();
  ~GnssEphemerisMsg();

  void map(GnssEphemeris *data);
  void unmap();
  GnssEphemeris* mutable_data();
  const GnssEphemeris& data() const;

  bool has_gnss_type() const;
  void clear_gnss_type();
  ::apollo::localization::msf::GnssType gnss_type() const;
  void set_gnss_type(::apollo::localization::msf::GnssType value);

  bool has_keppler_orbit() const;
  void clear_keppler_orbit();
  const ::apollo::localization::msf::KepplerOrbitMsg& keppler_orbit();
  ::apollo::localization::msf::KepplerOrbitMsg* mutable_keppler_orbit();

  bool has_glonass_orbit() const;
  void clear_glonass_orbit();
  const ::apollo::localization::msf::GlonassOrbitMsg& glonass_orbit();
  ::apollo::localization::msf::GlonassOrbitMsg* mutable_glonass_orbit();
 private:
  GnssEphemeris *data_;
  bool is_mapped_;
  KepplerOrbitMsg *keppler_orbit_msg_;
  GlonassOrbitMsg *glonass_orbit_msg_;
};

class SatDirCosine;
class SatDirCosineMsg {
 public:
  SatDirCosineMsg();
  ~SatDirCosineMsg();

  void map(SatDirCosine* data);
  void unmap();
  SatDirCosine* mutable_data();
  const SatDirCosine& data() const;

  bool has_sat_prn() const;
  void clear_sat_prn();
  uint32 sat_prn() const;
  void set_sat_prn(uint32 value);

  bool has_sat_sys() const;
  void clear_sat_sys();
  uint32 sat_sys() const;
  void set_sat_sys(uint32 value);

  bool has_cosine_x() const;
  void clear_cosine_x();
  double cosine_x() const;
  void set_cosine_x(double value);

  bool has_cosine_y() const;
  void clear_cosine_y();
  double cosine_y() const;
  void set_cosine_y(double value);

  bool has_cosine_z() const;
  void clear_cosine_z();
  double cosine_z() const;
  void set_cosine_z(double value);
 private:
  SatDirCosine *data_;
  bool is_mapped_;
};

class GnssPntResult;
class GnssPntResultMsg {
 public:
  GnssPntResultMsg();
  ~GnssPntResultMsg();

  void map(GnssPntResult *data);
  void unmap();
  GnssPntResult *mutable_data();
  const GnssPntResult& data() const;

  bool has_receiver_id() const;
  void clear_receiver_id();
  uint32 receiver_id() const;
  void set_receiver_id(uint32 value);

  bool has_time_type() const;
  void clear_time_type();
  ::apollo::localization::msf::GnssTimeType time_type() const;
  void set_time_type(::apollo::localization::msf::GnssTimeType value);

  bool has_gnss_week() const;
  void clear_gnss_week();
  uint32 gnss_week() const;
  void set_gnss_week(uint32 value);

  bool has_gnss_second_s() const;
  void clear_gnss_second_s();
  double gnss_second_s() const;
  void set_gnss_second_s(double value);

  bool has_pnt_type() const;
  void clear_pnt_type();
  ::apollo::localization::msf::PntType pnt_type() const;
  void set_pnt_type(::apollo::localization::msf::PntType value);

  bool has_pos_x_m() const;
  void clear_pos_x_m();
  double pos_x_m() const;
  void set_pos_x_m(double value);

  bool has_pos_y_m() const;
  void clear_pos_y_m();
  double pos_y_m() const;
  void set_pos_y_m(double value);

  bool has_pos_z_m() const;
  void clear_pos_z_m();
  double pos_z_m() const;
  void set_pos_z_m(double value);

  bool has_std_pos_x_m() const;
  void clear_std_pos_x_m();
  double std_pos_x_m() const;
  void set_std_pos_x_m(double value);

  bool has_std_pos_y_m() const;
  void clear_std_pos_y_m();
  double std_pos_y_m() const;
  void set_std_pos_y_m(double value);

  bool has_std_pos_z_m() const;
  void clear_std_pos_z_m();
  double std_pos_z_m() const;
  void set_std_pos_z_m(double value);

  bool has_vel_x_m() const;
  void clear_vel_x_m();
  double vel_x_m() const;
  void set_vel_x_m(double value);

  bool has_vel_y_m() const;
  void clear_vel_y_m();
  double vel_y_m() const;
  void set_vel_y_m(double value);

  bool has_vel_z_m() const;
  void clear_vel_z_m();
  double vel_z_m() const;
  void set_vel_z_m(double value);

  bool has_std_vel_x_m() const;
  void clear_std_vel_x_m();
  double std_vel_x_m() const;
  void set_std_vel_x_m(double value);

  bool has_std_vel_y_m() const;
  void clear_std_vel_y_m();
  double std_vel_y_m() const;
  void set_std_vel_y_m(double value);

  bool has_std_vel_z_m() const;
  void clear_std_vel_z_m();
  double std_vel_z_m() const;
  void set_std_vel_z_m(double value);

  bool has_sovled_sat_num() const;
  void clear_sovled_sat_num();
  uint32 sovled_sat_num() const;
  void set_sovled_sat_num(uint32 value);

  int sat_dir_cosine_size() const;
  void clear_sat_dir_cosine();
  const ::apollo::localization::msf::SatDirCosineMsg& sat_dir_cosine(int index);
  ::apollo::localization::msf::SatDirCosineMsg* mutable_sat_dir_cosine(int index);
  ::apollo::localization::msf::SatDirCosineMsg* add_sat_dir_cosine();

  bool has_pdop() const;
  void clear_pdop();
  double pdop() const;
  void set_pdop(double value);

  bool has_hdop() const;
  void clear_hdop();
  double hdop() const;
  void set_hdop(double value);

  bool has_vdop() const;
  void clear_vdop();
  double vdop() const;
  void set_vdop(double value);

 private:
  GnssPntResult *data_;
  bool is_mapped_;
  SatDirCosineMsg *sat_dir_cosine_msg_;
};

}  // namespace msf
}  // namespace localization
}  // namespace apollo
