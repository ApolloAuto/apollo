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
 * @file gnss_msg_transfer.h
 * @brief The class of GnssMsgTransfer
 */

#ifndef MODULES_LOCALIZATION_MSF_GNSS_MSG_TRANSFER_H_
#define MODULES_LOCALIZATION_MSF_GNSS_MSG_TRANSFER_H_

#include "modules/drivers/gnss/proto/gnss_raw_observation.pb.h"
#include "include/gnss_struct.h"

/**
 * @namespace apollo::localization::msf
 * @brief apollo::localization::msf
 */
namespace apollo {
namespace localization {
namespace msf {

class GnssMagTransfer {
 public:
  static void Transfer(const apollo::drivers::gnss::BandObservation &in,
                       BandObservationMsg* out);

  static void Transfer(const apollo::drivers::gnss::SatelliteObservation &in,
                       SatelliteObservationMsg* out);

  static void Transfer(const apollo::drivers::gnss::EpochObservation &in,
                       EpochObservationMsg* out);

  static void Transfer(const apollo::drivers::gnss::KepplerOrbit &in,
                       KepplerOrbitMsg* out);

  static void Transfer(const apollo::drivers::gnss::GlonassOrbit &in,
                       GlonassOrbitMsg* out);

  static void Transfer(const apollo::drivers::gnss::GnssEphemeris &in,
                       GnssEphemerisMsg* out);
};

}  // namespace msf
}  // namespace localization
}  // namespace apollo

#endif  // MODULES_LOCALIZATION_MSF_GNSS_MSG_TRANSFER_H_
