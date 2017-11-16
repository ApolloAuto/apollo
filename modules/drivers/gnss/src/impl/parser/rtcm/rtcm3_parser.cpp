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

#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <vector>

#include <ros/ros.h>
#include "gnss/parser.h"
#include "proto/gnss_raw_observation.pb.h"
#include "rtcm/rtcm_decode.h"

namespace apollo {
namespace drivers {
namespace gnss {

// Anonymous namespace that contains helper constants and functions.
namespace {

template <typename T>
constexpr bool is_zero(T value) {
  return value == static_cast<T>(0);
}

}  // namespace

class Rtcm3Parser : public Parser {
 public:
  Rtcm3Parser(bool is_base_satation);
  virtual MessageType get_message(MessagePtr &message_ptr);

 private:
  void set_observation_time();
  bool set_station_position();
  void fill_keppler_orbit(eph_t &eph,
                          apollo::drivers::gnss::KepplerOrbit &keppler_orbit);
  void fill_glonass_orbit(geph_t &eph,
                          apollo::drivers::gnss::GlonassOrbit &keppler_orbit);
  bool process_observation();
  bool process_ephemerides();
  bool process_station_parameters();
  bool _init_flag;

  std::vector<uint8_t> _buffer;

  rtcm_t _rtcm;
  bool _is_base_station = false;

  apollo::drivers::gnss::GnssEphemeris _ephemeris;
  apollo::drivers::gnss::EpochObservation _observation;

  // std::map<int, adu::common::Point3D> _station_location;
};

Parser *Parser::create_rtcm_v3(bool is_base_station) {
  return new Rtcm3Parser(is_base_station);
}

Rtcm3Parser::Rtcm3Parser(bool is_base_station) {
  if (1 != init_rtcm(&_rtcm)) {
    _init_flag = true;
  } else {
    _init_flag = false;
  }

  _ephemeris.Clear();
  _observation.Clear();
  _is_base_station = is_base_station;
}

bool Rtcm3Parser::set_station_position() {
#if 0
  auto iter = _station_location.find(_rtcm.staid);
  if (iter == _station_location.end()) {
    ROS_WARN("Station %d has no location info.", _rtcm.staid);
    return false;
  }

  _observation.set_position_x(iter->second.x());
  _observation.set_position_y(iter->second.y());
  _observation.set_position_z(iter->second.z());
#endif
  return true;
}

void Rtcm3Parser::fill_keppler_orbit(
    eph_t &eph, apollo::drivers::gnss::KepplerOrbit &keppler_orbit) {
  keppler_orbit.set_week_num(eph.week);

  keppler_orbit.set_af0(eph.f0);
  keppler_orbit.set_af1(eph.f1);
  keppler_orbit.set_af2(eph.f2);

  keppler_orbit.set_iode(eph.iode);
  keppler_orbit.set_deltan(eph.deln);
  keppler_orbit.set_m0(eph.M0);
  keppler_orbit.set_e(eph.e);

  keppler_orbit.set_roota(std::sqrt(eph.A));

  keppler_orbit.set_toe(eph.toes);
  keppler_orbit.set_toc(eph.tocs);

  keppler_orbit.set_cic(eph.cic);
  keppler_orbit.set_crc(eph.crc);
  keppler_orbit.set_cis(eph.cis);
  keppler_orbit.set_crs(eph.crs);
  keppler_orbit.set_cuc(eph.cuc);
  keppler_orbit.set_cus(eph.cus);

  keppler_orbit.set_omega0(eph.OMG0);
  keppler_orbit.set_omega(eph.omg);
  keppler_orbit.set_i0(eph.i0);
  keppler_orbit.set_omegadot(eph.OMGd);
  keppler_orbit.set_idot(eph.idot);

  // keppler_orbit.set_codesonL2channel(eph.);
  keppler_orbit.set_l2pdataflag(eph.flag);
  keppler_orbit.set_accuracy(eph.sva);
  keppler_orbit.set_health(eph.svh);
  keppler_orbit.set_tgd(eph.tgd[0]);
  keppler_orbit.set_iodc(eph.iodc);

  int prn = 0;
  satsys(eph.sat, &prn);
  keppler_orbit.set_sat_prn(prn);

  ROS_INFO_STREAM("Keppler orbit debuginfo:\r\n"
                  << keppler_orbit.DebugString());
}

void Rtcm3Parser::fill_glonass_orbit(
    geph_t &eph, apollo::drivers::gnss::GlonassOrbit &orbit) {
  orbit.set_position_x(eph.pos[0]);
  orbit.set_position_y(eph.pos[1]);
  orbit.set_position_z(eph.pos[2]);

  orbit.set_velocity_x(eph.vel[0]);
  orbit.set_velocity_y(eph.vel[1]);
  orbit.set_velocity_z(eph.vel[2]);

  orbit.set_accelerate_x(eph.acc[0]);
  orbit.set_accelerate_y(eph.acc[1]);
  orbit.set_accelerate_z(eph.acc[2]);

  orbit.set_health(eph.svh);
  orbit.set_clock_offset(-eph.taun);
  orbit.set_clock_drift(eph.gamn);
  orbit.set_infor_age(eph.age);

  orbit.set_frequency_no(eph.frq);
  // orbit.set_toe(eph.toe.time + eph.toe.sec);
  // orbit.set_tk(eph.tof.time + eph.tof.sec);

  int week = 0;
  double second = 0.0;

  second = time2gpst(eph.toe, &week);
  orbit.set_week_num(week);
  orbit.set_week_second_s(second);
  orbit.set_toe(second);

  second = time2gpst(eph.tof, &week);
  orbit.set_tk(second);

  orbit.set_gnss_time_type(apollo::drivers::gnss::GnssTimeType::GPS_TIME);

  int prn = 0;
  satsys(eph.sat, &prn);
  orbit.set_slot_prn(prn);
}

void Rtcm3Parser::set_observation_time() {
  int week = 0;
  double second = 0.0;

  second = time2gpst(_rtcm.time, &week);

  _observation.set_gnss_time_type(apollo::drivers::gnss::GPS_TIME);
  _observation.set_gnss_week(week);
  _observation.set_gnss_second_s(second);
}

Parser::MessageType Rtcm3Parser::get_message(MessagePtr &message_ptr) {
  if (_data == nullptr) {
    return MessageType::NONE;
  }

  int status = 0;
  while (_data < _data_end) {
    status = input_rtcm3(&_rtcm, *_data++);  // parse data use rtklib

    switch (status) {
      case 1:  // observation data
        if (process_observation()) {
          message_ptr = &_observation;
          return MessageType::OBSERVATION;
        }
        break;

      case 2:  // ephemeris
        if (process_ephemerides()) {
          message_ptr = &_ephemeris;
          return MessageType::EPHEMERIDES;
        }
        break;

      case 5:
        process_station_parameters();
        break;

      case 10:  // ssr messages
      default:
        break;
    }
  }

  return MessageType::NONE;
}

bool Rtcm3Parser::process_observation() {
  ROS_INFO("======Observation message.");
  ROS_INFO("Message type %d.", _rtcm.message_type);
  ROS_INFO_STREAM("Is base station: " << _is_base_station);
  ROS_INFO("Observation number %d.", _rtcm.obs.n);
  ROS_INFO("Station Id %d.", _rtcm.staid);
  ROS_INFO("time %ld.", _rtcm.time.time);
  ROS_INFO("sec %f.", _rtcm.time.sec);

  if (_rtcm.obs.n == 0) {
    ROS_WARN("Obs is zero.");
  }

  _observation.Clear();
  set_station_position();
  if (!_is_base_station) {
    _observation.set_receiver_id(0);
  } else {
    _observation.set_receiver_id(_rtcm.staid + 0x80000000);
  }

  // set time
  set_observation_time();

  // set satellite obs
  _observation.set_sat_obs_num(_rtcm.obs.n);
  _observation.set_health_flag(_rtcm.stah);

  for (int i = 0; i < _rtcm.obs.n; ++i) {
    int prn = 0;
    int sys = 0;

    sys = satsys(_rtcm.obs.data[i].sat, &prn);

    ROS_INFO("sat %d, receiver %d.", _rtcm.obs.data[i].sat,
             _rtcm.obs.data[i].rcv);
    ROS_INFO("sys %d, prn %d.", sys, prn);

    apollo::drivers::gnss::GnssType gnss_type;

    // transform sys to local sys type
    if (!gnss_sys_type(sys, gnss_type)) {
      return false;
    }

    auto sat_obs = _observation.add_sat_obs();  // create obj
    sat_obs->set_sat_prn(prn);
    sat_obs->set_sat_sys(gnss_type);

    int j = 0;

    for (j = 0; j < NFREQ + NEXOBS; ++j) {
      if (is_zero(_rtcm.obs.data[i].L[j])) {
        break;
      }

      apollo::drivers::gnss::GnssBandID baud_id;
      if (!gnss_baud_id(gnss_type, j, baud_id)) {
        break;
      }

      double freq = 0;
      gnss_frequence(baud_id, freq);

      auto band_obs = sat_obs->add_band_obs();
      if (_rtcm.obs.data[i].code[i] == CODE_L1C) {
        band_obs->set_pseudo_type(
            apollo::drivers::gnss::PseudoType::CORSE_CODE);
      } else if (_rtcm.obs.data[i].code[i] == CODE_L1P) {
        band_obs->set_pseudo_type(
            apollo::drivers::gnss::PseudoType::PRECISION_CODE);
      } else {
        ROS_INFO("Message type %d.", _rtcm.message_type);
        ROS_INFO("Code %d, in seq %d, gnss type %d.", _rtcm.obs.data[i].code[i],
                 j, static_cast<int>(gnss_type));
      }

      band_obs->set_band_id(baud_id);
      band_obs->set_frequency_value(freq);
      band_obs->set_pseudo_range(_rtcm.obs.data[i].P[j]);
      band_obs->set_carrier_phase(_rtcm.obs.data[i].L[j]);
      band_obs->set_loss_lock_index(_rtcm.obs.data[i].SNR[j]);
      band_obs->set_doppler(_rtcm.obs.data[i].D[j]);
      band_obs->set_snr(_rtcm.obs.data[i].SNR[j]);
    }
    ROS_INFO("Baud obs num %d.", j);
    sat_obs->set_band_obs_num(j);
  }

  ROS_INFO_STREAM("Obeservation debuginfo:\r\n" << _observation.DebugString());
  return true;
}

bool Rtcm3Parser::process_ephemerides() {
  apollo::drivers::gnss::GnssType gnss_type;

  ROS_INFO("======Ephemeris Message.");
  ROS_INFO("Message type %d.", _rtcm.message_type);
  ROS_INFO_STREAM("Is base station: " << _is_base_station);
  if (!gnss_sys(_rtcm.message_type, gnss_type)) {
    ROS_INFO("Failed get gnss type from message type %d.", _rtcm.message_type);
    return false;
    ;
  }

  apollo::drivers::gnss::GnssTimeType time_type;
  gnss_time_type(gnss_type, time_type);

  ROS_INFO("Gnss sys %d ephemeris info.", static_cast<int>(gnss_type));

  _ephemeris.Clear();
  _ephemeris.set_gnss_type(gnss_type);

  if (gnss_type == apollo::drivers::gnss::GnssType::GLO_SYS) {
    auto obit = _ephemeris.mutable_glonass_orbit();
    obit->set_gnss_type(gnss_type);
    obit->set_gnss_time_type(time_type);
    fill_glonass_orbit(_rtcm.nav.geph[_rtcm.ephsat - 1], *obit);
  } else {
    auto obit = _ephemeris.mutable_keppler_orbit();
    obit->set_gnss_type(gnss_type);
    obit->set_gnss_time_type(time_type);
    fill_keppler_orbit(_rtcm.nav.eph[_rtcm.ephsat - 1], *obit);
  }

  ROS_INFO_STREAM("Ephemeris debuginfo:\r\n" << _ephemeris.DebugString());
  return true;
}

bool Rtcm3Parser::process_station_parameters() {
  // station pose/ant parameters, set pose.
  ROS_INFO("======Station parameters message.");
  ROS_INFO("Message type %d.", _rtcm.message_type);
  ROS_INFO_STREAM("Is base station: " << _is_base_station);
  ROS_INFO("Station receiver number %s.", _rtcm.sta.recsno);
  ROS_INFO("Station pose (%f, %f, %f).", _rtcm.sta.pos[0], _rtcm.sta.pos[1],
           _rtcm.sta.pos[2]);

#if 0
  // update station location
  auto iter = _station_location.find(_rtcm.staid);
  if (iter == _station_location.end()) {
    adu::common::Point3D point;
    ROS_INFO("Add pose for station id: %d.", _rtcm.staid);
    point.set_x(_rtcm.sta.pos[0]);
    point.set_y(_rtcm.sta.pos[1]);
    point.set_z(_rtcm.sta.pos[2]);
    _station_location.insert(std::make_pair(_rtcm.staid, point));
  } else {
    iter->second.set_x(_rtcm.sta.pos[0]);
    iter->second.set_y(_rtcm.sta.pos[1]);
    iter->second.set_z(_rtcm.sta.pos[2]);
  }
#endif
  return true;
}

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
