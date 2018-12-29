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
#include "modules/prediction/common/prediction_gflags.h"

namespace apollo {
namespace prediction {
namespace network {

void CruiseModel::Run(const std::vector<Eigen::MatrixXf>& inputs,
                      Eigen::MatrixXf* output) const {
  // inputs = {lane_feature, obs_feature}
  CHECK_EQ(inputs.size(), 2);
  output->resize(1, 2);

  // Step 1: Run lane feature conv 1d
  Eigen::MatrixXf lane_conv1d_0_output;
  lane_conv1d_0_->Run({inputs[0]}, &lane_conv1d_0_output);
  Eigen::MatrixXf lane_activation_1_output;
  lane_activation_1_->Run({lane_conv1d_0_output}, &lane_activation_1_output);
  Eigen::MatrixXf lane_conv1d_2_output;
  lane_conv1d_2_->Run({lane_activation_1_output}, &lane_conv1d_2_output);

  // Step 2: Run lane feature max pool 1d
  Eigen::MatrixXf lane_maxpool1d_output;
  lane_maxpool1d_->Run({lane_conv1d_2_output}, &lane_maxpool1d_output);
  Eigen::MatrixXf lane_maxpool1d_flat = FlattenMatrix(lane_maxpool1d_output);

  // Step 3: Run lane feature avg pool 1d
  Eigen::MatrixXf lane_avgpool1d_output;
  lane_avgpool1d_->Run({lane_conv1d_2_output}, &lane_avgpool1d_output);
  Eigen::MatrixXf lane_avgpool1d_flat = FlattenMatrix(lane_avgpool1d_output);

  Eigen::MatrixXf lane_feature;
  concatenate_->Run({lane_maxpool1d_flat, lane_avgpool1d_flat}, &lane_feature);

  // Step 4: Run obstacle feature fully connected
  Eigen::MatrixXf obs_linear_0_output;
  obs_linear_0_->Run({inputs[1]}, &obs_linear_0_output);
  Eigen::MatrixXf obs_activation_1_output;
  obs_activation_1_->Run({obs_linear_0_output}, &obs_activation_1_output);
  Eigen::MatrixXf obs_linear_3_output;
  obs_linear_3_->Run({obs_activation_1_output}, &obs_linear_3_output);
  Eigen::MatrixXf obs_feature;
  obs_activation_4_->Run({obs_linear_3_output}, &obs_feature);

  // Step 5: Concatenate [lane_feature, obstacle_feature]
  Eigen::MatrixXf feature_values;
  concatenate_->Run({lane_feature, obs_feature}, &feature_values);

  // Step 6: Get classification result
  Eigen::MatrixXf classify_linear_0_output;
  classify_linear_0_->Run({feature_values}, &classify_linear_0_output);
  Eigen::MatrixXf classify_activation_1_output;
  classify_activation_1_->Run({classify_linear_0_output},
                              &classify_activation_1_output);
  Eigen::MatrixXf classify_linear_3_output;
  classify_linear_3_->Run({classify_activation_1_output},
                          &classify_linear_3_output);
  Eigen::MatrixXf classify_activation_4_output;
  classify_activation_4_->Run({classify_linear_3_output},
                              &classify_activation_4_output);
  Eigen::MatrixXf classify_linear_6_output;
  classify_linear_6_->Run({classify_activation_4_output},
                          &classify_linear_6_output);
  Eigen::MatrixXf classify_activation_7_output;
  classify_activation_7_->Run({classify_linear_6_output},
                              &classify_activation_7_output);
  Eigen::MatrixXf classify_linear_9_output;
  classify_linear_9_->Run({classify_activation_7_output},
                          &classify_linear_9_output);
  Eigen::MatrixXf classify_activation_10_output;
  classify_activation_10_->Run({classify_linear_9_output},
                               &classify_activation_10_output);

  CHECK_EQ(classify_activation_10_output.cols(), 1);
  float probability = classify_activation_10_output(0, 0);
  (*output)(0, 0) = probability;

  // Step 7: Get regression result
  if (probability < FLAGS_lane_sequence_threshold_cruise ||
      !FLAGS_enable_cruise_regression) {
    (*output)(0, 1) = static_cast<float>(FLAGS_time_to_center_if_not_reach);
    return;
  }

  Eigen::MatrixXf feature_values_regress;
  concatenate_->Run({feature_values, classify_linear_9_output},
                    &feature_values_regress);

  Eigen::MatrixXf regress_linear_0_output;
  regress_linear_0_->Run({feature_values_regress}, &regress_linear_0_output);

  Eigen::MatrixXf regress_activation_1_output;
  regress_activation_1_->Run({regress_linear_0_output},
                             &regress_activation_1_output);
  Eigen::MatrixXf regress_linear_3_output;
  regress_linear_3_->Run({regress_activation_1_output},
                          &regress_linear_3_output);
  Eigen::MatrixXf regress_activation_4_output;
  regress_activation_4_->Run({regress_linear_3_output},
                             &regress_activation_4_output);
  Eigen::MatrixXf regress_linear_6_output;
  regress_linear_6_->Run({regress_activation_4_output},
                         &regress_linear_6_output);
  Eigen::MatrixXf regress_activation_7_output;
  regress_activation_7_->Run({regress_linear_6_output},
                             &regress_activation_7_output);
  Eigen::MatrixXf regress_linear_9_output;
  regress_linear_9_->Run({regress_activation_7_output},
                         &regress_linear_9_output);
  Eigen::MatrixXf regress_activation_10_output;
  regress_activation_10_->Run({regress_linear_9_output},
                              &regress_activation_10_output);

  CHECK_EQ(classify_activation_10_output.cols(), 1);
  float time_to_lane_center = regress_activation_10_output(0, 0);
  (*output)(0, 1) = time_to_lane_center;
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
  lane_conv1d_0_->Load(lane_conv1d_param.conv1d_0());
  lane_activation_1_->Load(lane_conv1d_param.activation_1());
  lane_conv1d_2_->Load(lane_conv1d_param.conv1d_2());

  // Load MaxPool1dParameter
  const auto& lane_maxpool1d_param =
      cruise_model_parameter.lane_feature_maxpool();
  lane_maxpool1d_->Load(lane_maxpool1d_param);

  // Load AvgPool1dParameter
  const auto& lane_avgpool1d_param =
      cruise_model_parameter.lane_feature_avgpool();
  lane_avgpool1d_->Load(lane_avgpool1d_param);

  // Load ObsFeatureFCParameter
  const auto& obs_fc_param = cruise_model_parameter.obs_feature_fc();
  obs_linear_0_->Load(obs_fc_param.linear_0());
  obs_activation_1_->Load(obs_fc_param.activation_1());
  obs_linear_3_->Load(obs_fc_param.linear_3());
  obs_activation_4_->Load(obs_fc_param.activation_4());

  // Load ClassifyParameter
  const auto& classify_param = cruise_model_parameter.classify();
  classify_linear_0_->Load(classify_param.linear_0());
  classify_activation_1_->Load(classify_param.activation_1());
  classify_linear_3_->Load(classify_param.linear_3());
  classify_activation_4_->Load(classify_param.activation_4());
  classify_linear_6_->Load(classify_param.linear_6());
  classify_activation_7_->Load(classify_param.activation_7());
  classify_linear_9_->Load(classify_param.linear_9());
  classify_activation_10_->Load(classify_param.activation_10());

  // Load RegressParameter
  const auto& regress_param = cruise_model_parameter.regress();
  regress_linear_0_->Load(regress_param.linear_0());
  regress_activation_1_->Load(regress_param.activation_1());
  regress_linear_3_->Load(regress_param.linear_3());
  regress_activation_4_->Load(regress_param.activation_4());
  regress_linear_6_->Load(regress_param.linear_6());
  regress_activation_7_->Load(regress_param.activation_7());
  regress_linear_9_->Load(regress_param.linear_9());
  regress_activation_10_->Load(regress_param.activation_10());

  return true;
}

}  // namespace network
}  // namespace prediction
}  // namespace apollo
