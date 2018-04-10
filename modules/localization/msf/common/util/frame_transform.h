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

#ifndef MODULES_LOCALIZATION_MSF_COMMON_FRAME_TRANSFORM_H_
#define MODULES_LOCALIZATION_MSF_COMMON_FRAME_TRANSFORM_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdio>

namespace apollo {
namespace localization {
namespace msf {

/**@brief the UTM coordinate struct including x and y. */
struct UTMCoor {
  UTMCoor() : x(0.0), y(0.0) {}
  double x;
  double y;
};

/**@brief the WGS84 coordinate struct */
/* including log(longitude) and lat(latitude). */
struct WGS84Corr {
  WGS84Corr() : log(0.0), lat(0.0) {}
  double log;      // longitude
  double lat;      // latitude
};

void LatlonToUtmXY(const double lon, const double lat, UTMCoor *xy);

void UtmXYToLatlon(const double x, const double y, const int zone,
                     const bool southhemi, WGS84Corr *latlon);

void XYZToBlh(const Eigen::Vector3d &xyz, Eigen::Vector3d *blh);

void BlhToXYZ(const Eigen::Vector3d &blh, Eigen::Vector3d *xyz);

}   // namespace msf
}   // namespace localization
}   // namespace apollo

#endif  // MODULES_LOCALIZATION_MSF_COMMON_FRAME_TRANSFORM_H_
