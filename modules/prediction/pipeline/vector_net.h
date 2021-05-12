/******************************************************************************
 * Copyright 2021 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
 *implied. See the License for the specific language governing
 *permissions and limitations under the License.
 *****************************************************************************/

#pragma once

#include <vector>

#include "modules/common/util/point_factory.h"
#include "modules/prediction/common/prediction_system_gflags.h"
#include "modules/prediction/proto/vector_net.pb.h"

namespace apollo {
namespace prediction {

using FeatureVector = std::vector<std::vector<std::vector<double>>>;
using PidVector = std::vector<std::vector<double>>;

class VectorNet {
 public:
  VectorNet() = default;

  ~VectorNet() = default;

  bool query(const common::PointENU& center_point, const double obstacle_phi,
             FeatureVector* const feature_ptr, PidVector* const p_id_ptr);

  bool offline_query(const double obstacle_x, const double obstacle_y,
                     const double obstacle_phi);

 private:
  void GetRoads(const common::PointENU& center_point,
                const double obstacle_phi,
                FeatureVector* const feature_ptr, PidVector* const p_id_ptr);
  void GetLanes(const common::PointENU& center_point,
                           const double obstacle_phi,
                           FeatureVector* const feature_ptr,
                           PidVector* const p_id_ptr);
  void GetJunctions(const common::PointENU& center_point,
                    const double obstacle_phi,
                    FeatureVector* const feature_ptr,
                    PidVector* const p_id_ptr);
  void GetCrosswalks(const common::PointENU& center_point,
                    const double obstacle_phi,
                    FeatureVector* const feature_ptr,
                    PidVector* const p_id_ptr);
  int count_ = 0;
};

}  // namespace prediction
}  // namespace apollo
