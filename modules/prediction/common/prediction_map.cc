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

#include "modules/prediction/common/prediction_map.h"
#include "modules/prediction/common/prediction_gflags.h"
#include "modules/map/hdmap/hdmap.h"

namespace apollo {
namespace prediction {

using apollo::hdmap::LaneInfo;

PredictionMap::PredictionMap() : hdmap_(nullptr) {
  LoadMap();
}

PredictionMap::~PredictionMap() {
  Clear();
}

void PredictionMap::LoadMap() {
  hdmap_.reset(new apollo::hdmap::HDMapImpl());
  CHECK(hdmap_ != nullptr);
  hdmap_->load_map_from_file(FLAGS_map_file);
  ADEBUG << "Load map file: " << FLAGS_map_file;
}

void PredictionMap::Clear() {
  hdmap_.reset();
}

}  // namespace prediction
}  // namespace apollo
