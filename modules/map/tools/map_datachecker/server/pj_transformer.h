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
#pragma once

#include <proj_api.h>

namespace apollo {
namespace hdmap {

class PJTransformer {
 public:
  explicit PJTransformer(int zone_id = 50);
  ~PJTransformer();
  int LatlongToUtm(int64_t point_count, int point_offset, double *x, double *y,
                   double *z);

 private:
  projPJ pj_latlong_;
  projPJ pj_utm_;
};

}  // namespace hdmap
}  // namespace apollo
