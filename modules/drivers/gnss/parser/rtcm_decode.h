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

#include "modules/drivers/gnss/proto/gnss_raw_observation.pb.h"
#include "modules/drivers/gnss/third_party/rtklib.h"

namespace apollo {
namespace drivers {
namespace gnss {

static inline int baud_obs_num(int type) {
  switch (type) {
    case 1002:  // gps L1 only
      return 1;

    case 1004:  // gps L1/L1
      return 2;

    case 1010:  // glonass L1 only
      return 0;

    case 1012:  // glonass L1 & L2
      return 2;

    case 1125:
    case 1005:  // station arp, for pose, no antenna height
    case 1006:  // station arp, with antenna height
    case 1007:  // antenna descriptor
    case 1008:  // antenna descriptor&  serial number
    case 1019:  // gps ephemerides
    case 1020:  // glonass ephemerides
    case 1033:  // receiver and antenna descriptor, for station
    case 1044:  // qzss ephemerides
    case 1045:  // galileo
    default:
      return 0;
  }
}

static inline bool gnss_sys(int message_type,
                            apollo::drivers::gnss::GnssType* gnss_type) {
  switch (message_type) {
    case 1019:  // gps ephemerides
      *gnss_type = apollo::drivers::gnss::GnssType::GPS_SYS;
      break;

    case 1020:  // glonass ephemerides
      *gnss_type = apollo::drivers::gnss::GnssType::GLO_SYS;
      break;

    case 1045:  // galileo ephemerides
      *gnss_type = apollo::drivers::gnss::GnssType::GAL_SYS;
      break;

    case 1047:  // beidou ephemerides
      *gnss_type = apollo::drivers::gnss::GnssType::BDS_SYS;
      break;

    case 1044:  // qzss ephemerides
    default:
      return false;
  }
  return true;
}

static inline bool gnss_sys_type(int sys_id,
                                 apollo::drivers::gnss::GnssType* gnss_type) {
  switch (sys_id) {
    case SYS_GPS:
      *gnss_type = apollo::drivers::gnss::GnssType::GPS_SYS;
      break;

    case SYS_CMP:
      *gnss_type = apollo::drivers::gnss::GnssType::BDS_SYS;
      break;

    case SYS_GLO:
      *gnss_type = apollo::drivers::gnss::GnssType::GLO_SYS;
      break;

    case SYS_GAL:
      *gnss_type = apollo::drivers::gnss::GnssType::GAL_SYS;
      break;

    default:
      AINFO << "Not support sys id: " << sys_id;
      return false;
  }
  return true;
}

static inline bool gnss_baud_id(apollo::drivers::gnss::GnssType sys_type,
                                int seq,
                                apollo::drivers::gnss::GnssBandID* baud_id) {
  switch (sys_type) {
    case apollo::drivers::gnss::GnssType::GPS_SYS:
      if (seq == 0) {
        *baud_id = apollo::drivers::gnss::GnssBandID::GPS_L1;
      } else if (seq == 1) {
        *baud_id = apollo::drivers::gnss::GnssBandID::GPS_L2;
      } else if (seq == 2) {
        *baud_id = apollo::drivers::gnss::GnssBandID::GPS_L5;
      } else {
        AINFO << "Not support gps baud seq : " << seq;
        return false;
      }
      break;

    case apollo::drivers::gnss::GnssType::BDS_SYS:
      if (seq == 0) {
        *baud_id = apollo::drivers::gnss::GnssBandID::BDS_B1;
      } else if (seq == 1) {
        *baud_id = apollo::drivers::gnss::GnssBandID::BDS_B2;
      } else if (seq == 2) {
        *baud_id = apollo::drivers::gnss::GnssBandID::BDS_B3;
      } else {
        AINFO << "Not support beidou baud seq : " << seq;
        return false;
      }
      break;

    case apollo::drivers::gnss::GnssType::GLO_SYS:
      if (seq == 0) {
        *baud_id = apollo::drivers::gnss::GnssBandID::GLO_G1;
      } else if (seq == 1) {
        *baud_id = apollo::drivers::gnss::GnssBandID::GLO_G2;
      } else {
        AINFO << "Not support beidou glonass seq : " << seq;
        return false;
      }
      break;

    default:
      AINFO << "Not support sys " << static_cast<int>(sys_type) << ", seq "
            << seq;
      return false;
  }
  return true;
}

static inline bool gnss_time_type(
    apollo::drivers::gnss::GnssType sys_type,
    apollo::drivers::gnss::GnssTimeType* time_type) {
  switch (sys_type) {
    case apollo::drivers::gnss::GnssType::GPS_SYS:
      *time_type = apollo::drivers::gnss::GnssTimeType::GPS_TIME;
      break;

    case apollo::drivers::gnss::GnssType::BDS_SYS:
      *time_type = apollo::drivers::gnss::GnssTimeType::BDS_TIME;
      break;

    case apollo::drivers::gnss::GnssType::GLO_SYS:
      *time_type = apollo::drivers::gnss::GnssTimeType::GLO_TIME;
      break;

    case apollo::drivers::gnss::GnssType::GAL_SYS:
      *time_type = apollo::drivers::gnss::GnssTimeType::GAL_TIME;
      break;

    default:
      AINFO << "Not support sys " << static_cast<int>(sys_type);
      return false;
  }
  return true;
}

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
