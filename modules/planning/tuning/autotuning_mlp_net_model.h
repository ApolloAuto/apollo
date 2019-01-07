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

#pragma once

#include <vector>

#include "cyber/common/macros.h"

#include "modules/prediction/network/net_model.h"

namespace apollo {
namespace planning {

class AutotuningMLPModel : public prediction::network::NetModel {
 public:
  AutotuningMLPModel() = default;
  virtual ~AutotuningMLPModel() = default;
  /**
   * @brief Compute the model output from inputs according to a defined layers'
   * flow
   * @param Inputs to the network
   * @param Output of the network will be returned
   */
  void Run(const std::vector<Eigen::MatrixXf>& inputs,
           Eigen::MatrixXf* output) const override;
};

}  // namespace planning
}  // namespace apollo
