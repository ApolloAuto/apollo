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

#include "modules/prediction/network/cruise_model/cruise_model.h"
#include "cyber/common/log.h"

namespace apollo {
namespace prediction {
namespace network {

using apollo::prediction::CruiseModelParameter;

void CruiseModel::Run(const std::vector<Eigen::MatrixXf>& inputs,
                      Eigen::MatrixXf* output) const {
  // TODO(kechxu) implement
  // inputs = {lane_feature, obs_feature}
  CHECK_EQ(inputs.size(), 2);

  // Step 1: Run lane feature conv 1d

  // Step 2: Run lane feature max pool 1d

  // Step 3: Run lane feature avg pool 1d

  // Step 4: Run obstacle feature fully connected

  // Step 5: Concatenate [lane_feature, obstacle_feature]

  // Step 6: Get classification result

  // Step 7: Get regression result

  // Step 8: Output
}

bool CruiseModel::LoadModel(
    const CruiseModelParameter& cruise_model_parameter) {
  CHECK(cruise_model_parameter.has_lane_feature_conv());
  CHECK(cruise_model_parameter.has_lane_feature_maxpool());
  CHECK(cruise_model_parameter.has_lane_feature_avgpool());
  CHECK(cruise_model_parameter.has_obs_feature_fc());
  CHECK(cruise_model_parameter.has_classify());
  CHECK(cruise_model_parameter.has_regress());

  // Load LaneFeatureConvParameter
  const auto& lane_conv1d_param = cruise_model_parameter.lane_feature_conv();
  lane_conv1d_0_.Load(lane_conv1d_param.conv1d_0());
  lane_activation_1_.Load(lane_conv1d_param.activation_1());
  lane_conv1d_2_.Load(lane_conv1d_param.conv1d_2());
  lane_activation_3_.Load(lane_conv1d_param.activation_3());
  lane_conv1d_4_.Load(lane_conv1d_param.conv1d_4());

  // Load MaxPool1dParameter
  const auto& lane_maxpool1d_param =
      cruise_model_parameter.lane_feature_maxpool();
  lane_maxpool1d_.Load(lane_maxpool1d_param);

  // Load AvgPool1dParameter
  const auto& lane_avgpool1d_param =
      cruise_model_parameter.lane_feature_avgpool();
  lane_avgpool1d_.Load(lane_avgpool1d_param);

  // Load ObsFeatureFCParameter
  const auto& obs_fc_param = cruise_model_parameter.obs_feature_fc();
  obs_linear_0_.Load(obs_fc_param.linear_0());
  obs_activation_1_.Load(obs_fc_param.activation_1());
  obs_linear_3_.Load(obs_fc_param.linear_3());
  obs_activation_4_.Load(obs_fc_param.activation_4());

  // Load ClassifyParameter
  const auto& classify_param = cruise_model_parameter.classify();
  classify_linear_0_.Load(classify_param.linear_0());
  classify_activation_1_.Load(classify_param.activation_1());
  classify_linear_3_.Load(classify_param.linear_3());
  classify_activation_4_.Load(classify_param.activation_4());
  classify_linear_6_.Load(classify_param.linear_6());
  classify_activation_7_.Load(classify_param.activation_7());
  classify_linear_9_.Load(classify_param.linear_9());
  classify_activation_10_.Load(classify_param.activation_10());

  // Load RegressParameter
  const auto& regress_param = cruise_model_parameter.regress();
  regress_linear_0_.Load(regress_param.linear_0());
  regress_activation_1_.Load(regress_param.activation_1());
  regress_linear_3_.Load(regress_param.linear_3());
  regress_activation_4_.Load(regress_param.activation_4());
  regress_linear_6_.Load(regress_param.linear_6());
  regress_activation_7_.Load(regress_param.activation_7());
  regress_linear_9_.Load(regress_param.linear_9());
  regress_activation_10_.Load(regress_param.activation_10());

  return true;
}

}  // namespace network
}  // namespace prediction
}  // namespace apollo
