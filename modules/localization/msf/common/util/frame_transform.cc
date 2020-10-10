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

#include "modules/localization/msf/common/util/frame_transform.h"

#include <string>

#include "absl/strings/str_cat.h"

namespace apollo {
namespace localization {
namespace msf {

bool FrameTransform::LatlonToUtmXY(double lon_rad, double lat_rad,
                                   UTMCoor *utm_xy) {
  projPJ pj_latlon;
  projPJ pj_utm;
  int zone = 0;
  zone = static_cast<int>((lon_rad * RAD_TO_DEG + 180) / 6) + 1;
  std::string latlon_src =
      "+proj=longlat +ellps=GRS80 +towgs84=0,0,0,0,0,0,0 +no_defs";
  std::string utm_dst =
      absl::StrCat("+proj=utm +zone=", zone, " +ellps=GRS80 +units=m +no_defs");
  if (!(pj_latlon = pj_init_plus(latlon_src.c_str()))) {
    return false;
  }
  if (!(pj_utm = pj_init_plus(utm_dst.c_str()))) {
    return false;
  }
  double longitude = lon_rad;
  double latitude = lat_rad;
  pj_transform(pj_latlon, pj_utm, 1, 1, &longitude, &latitude, nullptr);
  utm_xy->x = longitude;
  utm_xy->y = latitude;
  pj_free(pj_latlon);
  pj_free(pj_utm);
  return true;
}
bool FrameTransform::UtmXYToLatlon(double x, double y, int zone, bool southhemi,
                                   WGS84Corr *latlon) {
  projPJ pj_latlon;
  projPJ pj_utm;
  std::string latlon_src =
      "+proj=longlat +ellps=GRS80 +towgs84=0,0,0,0,0,0,0 +no_defs";
  std::string utm_dst =
      absl::StrCat("+proj=utm +zone=", zone, " +ellps=GRS80 +units=m +no_defs");
  if (!(pj_latlon = pj_init_plus(latlon_src.c_str()))) {
    return false;
  }
  if (!(pj_utm = pj_init_plus(utm_dst.c_str()))) {
    return false;
  }
  pj_transform(pj_utm, pj_latlon, 1, 1, &x, &y, nullptr);
  latlon->log = x;
  latlon->lat = y;
  pj_free(pj_latlon);
  pj_free(pj_utm);
  return true;
}

bool FrameTransform::XYZToBlh(const Vector3d &xyz, Vector3d *blh) {
  projPJ pj_xyz;
  projPJ pj_blh;
  std::string xyz_src = "+proj=geocent +datum=WGS84";
  std::string blh_dst = "+proj=latlong +datum=WGS84";
  if (!(pj_xyz = pj_init_plus(xyz_src.c_str()))) {
    return false;
  }
  if (!(pj_blh = pj_init_plus(blh_dst.c_str()))) {
    return false;
  }
  double x = xyz[0];
  double y = xyz[1];
  double z = xyz[2];
  pj_transform(pj_xyz, pj_blh, 1, 1, &x, &y, &z);
  (*blh)[0] = x;
  (*blh)[1] = y;
  (*blh)[2] = z;
  pj_free(pj_xyz);
  pj_free(pj_blh);
  return true;
}
bool FrameTransform::BlhToXYZ(const Vector3d &blh, Vector3d *xyz) {
  projPJ pj_xyz;
  projPJ pj_blh;
  std::string blh_src = "+proj=latlong +datum=WGS84";
  std::string xyz_dst = "+proj=geocent +datum=WGS84";

  if (!(pj_blh = pj_init_plus(blh_src.c_str()))) {
    return false;
  }
  if (!(pj_xyz = pj_init_plus(xyz_dst.c_str()))) {
    return false;
  }
  double longitude = blh[0];
  double latitude = blh[1];
  double height = blh[2];
  pj_transform(pj_blh, pj_xyz, 1, 1, &longitude, &latitude, &height);
  (*xyz)[0] = longitude;
  (*xyz)[1] = latitude;
  (*xyz)[2] = height;
  pj_free(pj_xyz);
  pj_free(pj_blh);
  return true;
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
