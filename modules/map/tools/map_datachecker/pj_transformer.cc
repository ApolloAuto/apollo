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
#include <iostream>
#include <sstream>
#include "modules/map/tools/map_datachecker/pj_transformer.h"

namespace apollo {
namespace hdmap {

PJTransformer::PJTransformer(int zone_id) {
  // init projPJ
  std::stringstream stream;
  stream << "+proj=utm +zone=" << zone_id << " +ellps=WGS84" << std::endl;
  _pj_utm = pj_init_plus(stream.str().c_str());
  if (_pj_utm == NULL) {
    std::cerr << "proj4 init failed!" << stream.str() << std::endl;
    return;
  }
  _pj_latlong = pj_init_plus("+proj=latlong +ellps=WGS84");
  if (_pj_latlong == NULL) {
    std::cerr << "proj4 pj_latlong init failed!";
    return;
  }
  std::cerr << "proj4 init success" << std::endl;
}

PJTransformer::~PJTransformer() {
  if (_pj_latlong) {
    pj_free(_pj_latlong);
    _pj_latlong = NULL;
  }
  if (_pj_utm) {
    pj_free(_pj_utm);
    _pj_utm = NULL;
  }
}
int PJTransformer::latlong_to_utm(int64_t point_count, int point_offset,
                  double *x, double *y, double *z) {
  if (!_pj_latlong || !_pj_utm) {
    std::cerr << "_pj_latlong:" << _pj_latlong
          << "_pj_utm:" << _pj_utm << std::endl;
    return -1;
  }
  return pj_transform(
    _pj_latlong, _pj_utm, point_count, point_offset, x, y, z);
}

}  // namespace hdmap
}  // namespace apollo
