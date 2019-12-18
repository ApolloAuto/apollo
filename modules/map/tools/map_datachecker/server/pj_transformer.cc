/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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
#include "modules/map/tools/map_datachecker/server/pj_transformer.h"

#include <iostream>
#include <sstream>

#include "cyber/cyber.h"

namespace apollo {
namespace hdmap {

PJTransformer::PJTransformer(int zone_id) {
  // init projPJ
  std::stringstream stream;
  stream << "+proj=utm +zone=" << zone_id << " +ellps=WGS84" << std::endl;
  pj_utm_ = pj_init_plus(stream.str().c_str());
  if (pj_utm_ == nullptr) {
    AERROR << "proj4 init failed!" << stream.str() << std::endl;
    return;
  }
  pj_latlong_ = pj_init_plus("+proj=latlong +ellps=WGS84");
  if (pj_latlong_ == nullptr) {
    AERROR << "proj4 pj_latlong init failed!";
    return;
  }
  AINFO << "proj4 init success" << std::endl;
}

PJTransformer::~PJTransformer() {
  if (pj_latlong_) {
    pj_free(pj_latlong_);
    pj_latlong_ = nullptr;
  }
  if (pj_utm_) {
    pj_free(pj_utm_);
    pj_utm_ = nullptr;
  }
}
int PJTransformer::LatlongToUtm(int64_t point_count, int point_offset,
                                double *x, double *y, double *z) {
  if (!pj_latlong_ || !pj_utm_) {
    AERROR << "pj_latlong_:" << pj_latlong_ << "pj_utm_:" << pj_utm_
           << std::endl;
    return -1;
  }
  return pj_transform(pj_latlong_, pj_utm_, point_count, point_offset, x, y, z);
}

}  // namespace hdmap
}  // namespace apollo
