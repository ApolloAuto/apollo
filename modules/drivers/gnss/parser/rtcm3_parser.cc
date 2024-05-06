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

#include "modules/drivers/gnss/parser/rtcm3_parser.h"

#include <utility>

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

Parser *Parser::CreateRtcmV3(bool is_base_station) {
  return new Rtcm3Parser(is_base_station);
}

Rtcm3Parser::Rtcm3Parser(bool is_base_station) {
  if (1 != init_rtcm(&rtcm_)) {
    init_flag_ = true;
  } else {
    init_flag_ = false;
  }

  ephemeris_.Clear();
  observation_.Clear();
  is_base_station_ = is_base_station;
}

bool Rtcm3Parser::SetStationPosition() {
  auto iter = station_location_.find(rtcm_.staid);
  if (iter == station_location_.end()) {
    AWARN << "Station " << rtcm_.staid << " has no location info.";
    return false;
  }

  observation_.set_position_x(iter->second.x);
  observation_.set_position_y(iter->second.y);
  observation_.set_position_z(iter->second.z);
  return true;
}

void Rtcm3Parser::FillKepplerOrbit(
    const eph_t &eph, apollo::drivers::gnss::KepplerOrbit *keppler_orbit) {
  keppler_orbit->set_week_num(eph.week);

  keppler_orbit->set_af0(eph.f0);
  keppler_orbit->set_af1(eph.f1);
  keppler_orbit->set_af2(eph.f2);

  keppler_orbit->set_iode(eph.iode);
  keppler_orbit->set_deltan(eph.deln);
  keppler_orbit->set_m0(eph.M0);
  keppler_orbit->set_e(eph.e);

  keppler_orbit->set_roota(std::sqrt(eph.A));

  keppler_orbit->set_toe(eph.toes);
  keppler_orbit->set_toc(eph.tocs);

  keppler_orbit->set_cic(eph.cic);
  keppler_orbit->set_crc(eph.crc);
  keppler_orbit->set_cis(eph.cis);
  keppler_orbit->set_crs(eph.crs);
  keppler_orbit->set_cuc(eph.cuc);
  keppler_orbit->set_cus(eph.cus);

  keppler_orbit->set_omega0(eph.OMG0);
  keppler_orbit->set_omega(eph.omg);
  keppler_orbit->set_i0(eph.i0);
  keppler_orbit->set_omegadot(eph.OMGd);
  keppler_orbit->set_idot(eph.idot);

  // keppler_orbit->set_codesonL2channel(eph.);
  keppler_orbit->set_l2pdataflag(eph.flag);
  keppler_orbit->set_accuracy(eph.sva);
  keppler_orbit->set_health(eph.svh);
  keppler_orbit->set_tgd(eph.tgd[0]);
  keppler_orbit->set_iodc(eph.iodc);

  int prn = 0;
  satsys(eph.sat, &prn);
  keppler_orbit->set_sat_prn(prn);
}

void Rtcm3Parser::FillGlonassOrbit(const geph_t &eph,
                                   apollo::drivers::gnss::GlonassOrbit *orbit) {
  orbit->set_position_x(eph.pos[0]);
  orbit->set_position_y(eph.pos[1]);
  orbit->set_position_z(eph.pos[2]);

  orbit->set_velocity_x(eph.vel[0]);
  orbit->set_velocity_y(eph.vel[1]);
  orbit->set_velocity_z(eph.vel[2]);

  orbit->set_accelerate_x(eph.acc[0]);
  orbit->set_accelerate_y(eph.acc[1]);
  orbit->set_accelerate_z(eph.acc[2]);

  orbit->set_health(eph.svh);
  orbit->set_clock_offset(-eph.taun);
  orbit->set_clock_drift(eph.gamn);
  orbit->set_infor_age(eph.age);

  orbit->set_frequency_no(eph.frq);
  // orbit->set_toe(eph.toe.time + eph.toe.sec);
  // orbit->set_tk(eph.tof.time + eph.tof.sec);

  int week = 0;

  double second = time2gpst(eph.toe, &week);
  orbit->set_week_num(week);
  orbit->set_week_second_s(second);
  orbit->set_toe(second);

  second = time2gpst(eph.tof, &week);
  orbit->set_tk(second);

  orbit->set_gnss_time_type(apollo::drivers::gnss::GnssTimeType::GPS_TIME);

  int prn = 0;
  satsys(eph.sat, &prn);
  orbit->set_slot_prn(prn);
}

void Rtcm3Parser::SetObservationTime() {
  int week = 0;
  double second = time2gpst(rtcm_.time, &week);
  observation_.set_gnss_time_type(apollo::drivers::gnss::GPS_TIME);
  observation_.set_gnss_week(week);
  observation_.set_gnss_second_s(second);
}

MessageType Rtcm3Parser::GetMessage(MessagePtr *message_ptr) {
  if (data_ == nullptr) {
    return MessageType::NONE;
  }

  while (data_ < data_end_) {
    const int status = input_rtcm3(&rtcm_, *data_++);  // parse data use rtklib

    switch (status) {
      case 1:  // observation data
        if (ProcessObservation()) {
          *message_ptr = &observation_;
          return MessageType::OBSERVATION;
        }
        break;

      case 2:  // ephemeris
        if (ProcessEphemerides()) {
          *message_ptr = &ephemeris_;
          return MessageType::EPHEMERIDES;
        }
        break;

      case 5:
        ProcessStationParameters();
        break;

      case 10:  // ssr messages
      default:
        break;
    }
  }

  return MessageType::NONE;
}

bool Rtcm3Parser::ProcessObservation() {
  if (rtcm_.obs.n == 0) {
    AWARN << "Obs is zero.";
  }

  observation_.Clear();
  SetStationPosition();
  if (!is_base_station_) {
    observation_.set_receiver_id(0);
  } else {
    observation_.set_receiver_id(rtcm_.staid + 0x80000000);
  }

  // set time
  SetObservationTime();

  // set satellite obs
  observation_.set_sat_obs_num(rtcm_.obs.n);
  observation_.set_health_flag(rtcm_.stah);

  for (int i = 0; i < rtcm_.obs.n; ++i) {
    int prn = 0;
    int sys = 0;

    sys = satsys(rtcm_.obs.data[i].sat, &prn);

    apollo::drivers::gnss::GnssType gnss_type;

    // transform sys to local sys type
    if (!gnss_sys_type(sys, &gnss_type)) {
      return false;
    }

    auto sat_obs = observation_.add_sat_obs();  // create obj
    sat_obs->set_sat_prn(prn);
    sat_obs->set_sat_sys(gnss_type);

    int j = 0;

    for (j = 0; j < NFREQ + NEXOBS; ++j) {
      if (is_zero(rtcm_.obs.data[i].L[j])) {
        break;
      }

      apollo::drivers::gnss::GnssBandID baud_id;
      if (!gnss_baud_id(gnss_type, j, &baud_id)) {
        break;
      }

      auto band_obs = sat_obs->add_band_obs();
      if (rtcm_.obs.data[i].code[i] == CODE_L1C) {
        band_obs->set_pseudo_type(
            apollo::drivers::gnss::PseudoType::CORSE_CODE);
      } else if (rtcm_.obs.data[i].code[i] == CODE_L1P) {
        band_obs->set_pseudo_type(
            apollo::drivers::gnss::PseudoType::PRECISION_CODE);
      } else {
        // AINFO << "Message type " << rtcm_.message_type;
      }

      band_obs->set_band_id(baud_id);
      band_obs->set_pseudo_range(rtcm_.obs.data[i].P[j]);
      band_obs->set_carrier_phase(rtcm_.obs.data[i].L[j]);
      band_obs->set_loss_lock_index(rtcm_.obs.data[i].SNR[j]);
      band_obs->set_doppler(rtcm_.obs.data[i].D[j]);
      band_obs->set_snr(rtcm_.obs.data[i].SNR[j]);
    }
    sat_obs->set_band_obs_num(j);
  }

  return true;
}

bool Rtcm3Parser::ProcessEphemerides() {
  apollo::drivers::gnss::GnssType gnss_type;

  if (!gnss_sys(rtcm_.message_type, &gnss_type)) {
    AINFO << "Failed get gnss type from message type " << rtcm_.message_type;
    return false;
  }

  apollo::drivers::gnss::GnssTimeType time_type;
  gnss_time_type(gnss_type, &time_type);

  AINFO << "Gnss sys " << static_cast<int>(gnss_type) << "ephemeris info.";

  ephemeris_.Clear();
  ephemeris_.set_gnss_type(gnss_type);

  if (gnss_type == apollo::drivers::gnss::GnssType::GLO_SYS) {
    auto obit = ephemeris_.mutable_glonass_orbit();
    obit->set_gnss_type(gnss_type);
    obit->set_gnss_time_type(time_type);
    FillGlonassOrbit(rtcm_.nav.geph[rtcm_.ephsat - 1], obit);
  } else {
    auto obit = ephemeris_.mutable_keppler_orbit();
    obit->set_gnss_type(gnss_type);
    obit->set_gnss_time_type(time_type);
    FillKepplerOrbit(rtcm_.nav.eph[rtcm_.ephsat - 1], obit);
  }

  return true;
}

bool Rtcm3Parser::ProcessStationParameters() {
  // station pose/ant parameters, set pose.

  // update station location
  auto iter = station_location_.find(rtcm_.staid);
  if (iter == station_location_.end()) {
    Point3D point;
    AINFO << "Add pose for station id: " << rtcm_.staid;
    point.x = rtcm_.sta.pos[0];
    point.y = rtcm_.sta.pos[1];
    point.z = rtcm_.sta.pos[2];
    station_location_.insert(std::make_pair(rtcm_.staid, point));
  } else {
    iter->second.x = rtcm_.sta.pos[0];
    iter->second.y = rtcm_.sta.pos[1];
    iter->second.z = rtcm_.sta.pos[2];
  }
  return true;
}

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
