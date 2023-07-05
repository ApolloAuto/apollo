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

#include "modules/prediction/network/rnn_model/rnn_model.h"

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
  Eigen::MatrixXf act1;
  layers_[6]->Run({lstm1, lstm2}, &merge);
  layers_[7]->Run({merge}, &dense1);
  layers_[8]->Run({dense1}, &bn1);
  layers_[9]->Run({bn1}, &act1);

  Eigen::MatrixXf dense2;
  Eigen::MatrixXf prob;
  layers_[10]->Run({act1}, &dense2);
  layers_[12]->Run({dense2}, &bn1);
  layers_[14]->Run({bn1}, &prob);

  Eigen::MatrixXf acc;
  layers_[11]->Run({act1}, &dense2);
  layers_[13]->Run({dense2}, &bn1);
  layers_[15]->Run({bn1}, &acc);

  output->resize(1, 2);
  *output << prob, acc;
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
