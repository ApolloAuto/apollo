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
#pragma once

#include <proj_api.h>

#include "Eigen/Geometry"

namespace apollo {
namespace localization {
namespace msf {

typedef Eigen::Vector3d Vector3d;

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
  double log;  // longitude
  double lat;  // latitude
};

class FrameTransform {
 public:
  static bool LatlonToUtmXY(double lon, double lat, UTMCoor *utm_xy);
  static bool UtmXYToLatlon(double x, double y, int zone, bool southhemi,
                            WGS84Corr *latlon);
  static bool XYZToBlh(const Vector3d &xyz, Vector3d *blh);
  static bool BlhToXYZ(const Vector3d &blh, Vector3d *xyz);

  //  static bool XyzToBlh(const Vector3d& xyz, Position *blh);
  //  static bool BlhToXyz(const Position& blh, Vector3d *xyz);
};

}  // namespace msf
}  // namespace localization
}  // namespace apollo
