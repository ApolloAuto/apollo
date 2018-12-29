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

#include <memory>
#include <vector>

#include "modules/prediction/network/net_model.h"
#include "modules/prediction/proto/cruise_model.pb.h"

namespace apollo {
namespace prediction {
namespace network {

class CruiseModel : public NetModel {
 public:
  /**
   * @brief Load cruise network parameters from a protobuf message
   * @param CruiseModelParameter is a protobuf message
   * @return True if successfully loaded, otherwise False
   */
  bool LoadModel(
      const apollo::prediction::CruiseModelParameter& cruise_model_parameter);

  /**
   * @brief Compute the model output from inputs according to a defined layers'
   * flow
   * @param Inputs to the network
   * @param Output of the network will be returned
   */
  void Run(const std::vector<Eigen::MatrixXf>& inputs,
           Eigen::MatrixXf* output) const override;

 private:
  // LaneFeatureConvParameter
  std::unique_ptr<Conv1d> lane_conv1d_0_ =
      std::unique_ptr<Conv1d>(new Conv1d());
  std::unique_ptr<Activation> lane_activation_1_ =
      std::unique_ptr<Activation>(new Activation());
  std::unique_ptr<Conv1d> lane_conv1d_2_ =
      std::unique_ptr<Conv1d>(new Conv1d());

  // MaxPool1dParameter
  std::unique_ptr<MaxPool1d> lane_maxpool1d_ =
      std::unique_ptr<MaxPool1d>(new MaxPool1d());

  // AvgPool1dParameter
  std::unique_ptr<AvgPool1d> lane_avgpool1d_ =
      std::unique_ptr<AvgPool1d>(new AvgPool1d());

  // ObsFeatureFCParameter
  std::unique_ptr<Dense> obs_linear_0_ =
      std::unique_ptr<Dense>(new Dense());
  std::unique_ptr<Activation> obs_activation_1_ =
      std::unique_ptr<Activation>(new Activation());
  std::unique_ptr<Dense> obs_linear_3_ =
      std::unique_ptr<Dense>(new Dense());
  std::unique_ptr<Activation> obs_activation_4_ =
      std::unique_ptr<Activation>(new Activation());

  // ClassifyParameter
  std::unique_ptr<Dense> classify_linear_0_ =
      std::unique_ptr<Dense>(new Dense());
  std::unique_ptr<Activation> classify_activation_1_ =
      std::unique_ptr<Activation>(new Activation());
  std::unique_ptr<Dense> classify_linear_3_ =
      std::unique_ptr<Dense>(new Dense());
  std::unique_ptr<Activation> classify_activation_4_ =
      std::unique_ptr<Activation>(new Activation());
  std::unique_ptr<Dense> classify_linear_6_ =
      std::unique_ptr<Dense>(new Dense());
  std::unique_ptr<Activation> classify_activation_7_ =
      std::unique_ptr<Activation>(new Activation());
  std::unique_ptr<Dense> classify_linear_9_ =
      std::unique_ptr<Dense>(new Dense());
  std::unique_ptr<Activation> classify_activation_10_ =
      std::unique_ptr<Activation>(new Activation());

  // RegressParameter
  std::unique_ptr<Dense> regress_linear_0_ =
      std::unique_ptr<Dense>(new Dense());
  std::unique_ptr<Activation> regress_activation_1_ =
      std::unique_ptr<Activation>(new Activation());
  std::unique_ptr<Dense> regress_linear_3_ =
      std::unique_ptr<Dense>(new Dense());
  std::unique_ptr<Activation> regress_activation_4_ =
      std::unique_ptr<Activation>(new Activation());
  std::unique_ptr<Dense> regress_linear_6_ =
      std::unique_ptr<Dense>(new Dense());
  std::unique_ptr<Activation> regress_activation_7_ =
      std::unique_ptr<Activation>(new Activation());
  std::unique_ptr<Dense> regress_linear_9_ =
      std::unique_ptr<Dense>(new Dense());
  std::unique_ptr<Activation> regress_activation_10_ =
      std::unique_ptr<Activation>(new Activation());

  // Concatenate
  std::unique_ptr<Concatenate> concatenate_ =
      std::unique_ptr<Concatenate>(new Concatenate());
};

}  // namespace network
}  // namespace prediction
}  // namespace apollo
