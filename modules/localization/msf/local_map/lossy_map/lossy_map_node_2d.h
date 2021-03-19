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

#include "modules/localization/msf/local_map/base_map/base_map_node.h"
#include "modules/localization/msf/local_map/lossy_map/lossy_map_matrix_2d.h"

namespace apollo {
namespace localization {
namespace msf {

class LossyMapNode2D : public BaseMapNode {
 public:
  LossyMapNode2D() : BaseMapNode(new LossyMapMatrix2D(), new ZlibStrategy()) {}
  ~LossyMapNode2D() {}
};

}  // namespace msf
}  // namespace localization
}  // namespace apollo
