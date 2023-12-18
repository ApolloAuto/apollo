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

#include "modules/planning/planning_base/learning_based/tuning/autotuning_mlp_net_model.h"

namespace apollo {
namespace planning {

void AutotuningMLPModel::Run(const std::vector<Eigen::MatrixXf>& inputs,
                             Eigen::MatrixXf* const output) const {
  Eigen::MatrixXf inp = inputs[0];
  Eigen::MatrixXf temp;
  for (size_t i = 0; i < layers_.size(); ++i) {
    layers_[i]->Run({inp}, &temp);
    inp = temp;
  }
  *output = temp;
}

}  // namespace planning
}  // namespace apollo
