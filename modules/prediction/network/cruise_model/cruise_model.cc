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

  cruise_model_parameter_.CopyFrom(cruise_model_parameter);
  return true;
}

}  // namespace network
}  // namespace prediction
}  // namespace apollo
