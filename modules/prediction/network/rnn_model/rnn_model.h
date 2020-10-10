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

/**
 * @namespace apollo::prediction::network
 * @brief apollo::prediction::network
 */
namespace apollo {
namespace prediction {
namespace network {

/**
 * @class RnnModel
 * @brief RnnModel is a derived class from NetModel, it has a specific layers
 * structure.
 */
class RnnModel : public NetModel {
 public:
  /**
   * @brief Compute the model output from inputs according to a defined layers'
   * flow
   * @param Inputs to the network
   * @param Output of the network will be returned
   */
  void Run(const std::vector<Eigen::MatrixXf>& inputs,
           Eigen::MatrixXf* output) const override;

  /**
   * @brief Set the internal state of a network model
   * @param A specified internal state in a vector of Eigen::MatrixXf
   */
  void SetState(const std::vector<Eigen::MatrixXf>& states) override;

  /**
   * @brief Access to the internal state of a network model
   * @return Internal state in a vector of Eigen::MatrixXf of the model
   */
  void State(std::vector<Eigen::MatrixXf>* states) const override;

  /**
   * @brief Set the internal state of a model
   * @param A specified internal state in a vector of Eigen::MatrixXf
   */
  void ResetState() const override;

  DECLARE_SINGLETON(RnnModel)
};

}  // namespace network
}  // namespace prediction
}  // namespace apollo
