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

#include "modules/prediction/evaluator/network/rnn_model.h"

#include <string>
#include <vector>

#include "Eigen/Dense"

#include "modules/common/log.h"
#include "modules/common/macro.h"
#include "modules/prediction/evaluator/network/layer/layer.h"

namespace apollo {
namespace prediction {
namespace network {

RnnModel::RnnModel() {}

void RnnModel::Run(const std::vector<Eigen::MatrixXf>& inputs,
                   Eigen::MatrixXf* output) const {
  Eigen::MatrixXf inp1;
  Eigen::MatrixXf inp2;
  layers_[0]->Run({inputs[0]}, &inp1);
  layers_[1]->Run({inputs[1]}, &inp2);

  Eigen::MatrixXf bn1;
  Eigen::MatrixXf bn2;
  layers_[2]->Run({inp1}, &bn1);
  layers_[3]->Run({inp2}, &bn2);

  Eigen::MatrixXf lstm1;
  Eigen::MatrixXf lstm2;
  layers_[4]->Run({bn1}, &lstm1);
  layers_[5]->Run({bn2}, &lstm2);

  Eigen::MatrixXf merge;
  Eigen::MatrixXf dense1;
  layers_[6]->Run({lstm1, lstm2}, &merge);
  layers_[7]->Run({merge}, &dense1);
  layers_[8]->Run({dense1}, &bn1);

  Eigen::MatrixXf act;
  Eigen::MatrixXf dense2;
  layers_[9]->Run({bn1}, &act);
  layers_[10]->Run({act}, &dense2);
  layers_[11]->Run({dense2}, &bn1);
  layers_[12]->Run({bn1}, output);  // sigmoid activation
}

bool RnnModel::VerifyModel() const {
  Eigen::MatrixXf obstacle_feature;
  Eigen::MatrixXf lane_feature;
  Eigen::MatrixXf output;

  ADEBUG << "Check Model";
  for (int i = 0; i < net_parameter_.verification_samples_size(); ++i) {
    VerificationSample sample = net_parameter_.verification_samples(i);
    CHECK_EQ(sample.features_size(), 2);
    if (!LoadTensor(sample.features(0), &obstacle_feature)) {
      AERROR << "Fail to load verification samples!";
      return false;
    }
    if (!LoadTensor(sample.features(1), &lane_feature)) {
      AERROR << "Fail to load verification samples!";
      return false;
    }
    this->Run({obstacle_feature, lane_feature}, &output);
    if (output.size() != 1) {
      AERROR << "output size != 1!";
      return false;
    }
    if (std::fabs(output(0, 0) - sample.probability()) > 1e-2) {
      AERROR << "predict: " << output(0, 0)
             << ",  actual: " << sample.probability();
      return false;
    }
    ResetState();
  }
  ADEBUG << "Success to validate the model.";
  return true;
}

void RnnModel::SetState(const std::vector<Eigen::MatrixXf>& states) {
  layers_[4]->SetState(states);
  layers_[5]->ResetState();
}

void RnnModel::State(std::vector<Eigen::MatrixXf>* states) const {
  layers_[4]->State(states);
}

void RnnModel::ResetState() const {
  layers_[4]->ResetState();
  layers_[5]->ResetState();
}

}  // namespace network
}  // namespace prediction
}  // namespace apollo
