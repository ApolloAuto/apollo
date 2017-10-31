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

#include <string>
#include <vector>

#include "Eigen/Dense"

#include "modules/prediction/evaluator/network/layer/util.h"
#include "modules/prediction/proto/network_layers.pb.h"

#ifndef MODULES_PREDICTION_EVALUATOR_NETWORK_LAYER_LAYER_H_
#define MODULES_PREDICTION_EVALUATOR_NETWORK_LAYER_LAYER_H_

/**
 * @namespace apollo::prediction::network
 * @brief apollo::prediction::network
 */
namespace apollo {
namespace prediction {
namespace network {

class Layer {
 public:
  virtual ~Layer() {}
  virtual bool Load(const apollo::prediction::LayerParameter& layer_pb);
  virtual void ResetState() {}
  virtual void SetState(const std::vector<Eigen::MatrixXf>& states) {}
  virtual void State(std::vector<Eigen::MatrixXf>* states) const {}

  virtual void Run(const std::vector<Eigen::MatrixXf>& inputs,
                   Eigen::MatrixXf* output) = 0;

  std::string Name() const { return name_; }
  int OrderNumber() const { return order_number_; }

 private:
  std::string name_;
  int order_number_;
};

class Dense : public Layer {
 public:
  virtual bool Load(const apollo::prediction::LayerParameter& layer_pb);
  virtual void Run(const std::vector<Eigen::MatrixXf>& inputs,
                   Eigen::MatrixXf* output);

 private:
  int units_;
  bool use_bias_;
  Eigen::MatrixXf weights_;
  Eigen::VectorXf bias_;
  std::function<float(float)> kactivation_;
};

class Activation : public Layer {
 public:
  virtual bool Load(const apollo::prediction::LayerParameter& layer_pb);
  virtual void Run(const std::vector<Eigen::MatrixXf>& inputs,
                   Eigen::MatrixXf* output);

 private:
  std::function<float(float)> kactivation_;
};

class BatchNormalization : public Layer {
 public:
  virtual bool Load(const apollo::prediction::LayerParameter& layer_pb);
  virtual void Run(const std::vector<Eigen::MatrixXf>& inputs,
                   Eigen::MatrixXf* output);

 private:
  Eigen::VectorXf mu_;
  Eigen::VectorXf sigma_;
  Eigen::VectorXf gamma_;
  Eigen::VectorXf beta_;
  float epsilon_;
  float momentum_;
  int axis_;
  bool center_;
  bool scale_;
};

class LSTM : public Layer {
 public:
  bool Load(const apollo::prediction::LayerParameter& layer_pb);
  virtual void Run(const std::vector<Eigen::MatrixXf>& inputs,
                   Eigen::MatrixXf* output);
  void Step(const Eigen::MatrixXf& input, Eigen::MatrixXf* output,
            Eigen::MatrixXf* ht_1, Eigen::MatrixXf* ct_1);
  virtual void ResetState();
  virtual void SetState(const std::vector<Eigen::MatrixXf>& states);
  virtual void State(std::vector<Eigen::MatrixXf>* states) const;

 private:
  Eigen::MatrixXf wi_;
  Eigen::MatrixXf wf_;
  Eigen::MatrixXf wc_;
  Eigen::MatrixXf wo_;
  Eigen::VectorXf bi_;
  Eigen::VectorXf bf_;
  Eigen::VectorXf bc_;
  Eigen::VectorXf bo_;

  Eigen::MatrixXf r_wi_;
  Eigen::MatrixXf r_wf_;
  Eigen::MatrixXf r_wc_;
  Eigen::MatrixXf r_wo_;

  Eigen::MatrixXf ht_1_;
  Eigen::MatrixXf ct_1_;
  std::function<float(float)> kactivation_;
  std::function<float(float)> krecurrent_activation_;
  int units_;
  bool return_sequences_;
  bool stateful_;
  bool use_bias_;
  bool unit_forget_bias_;
};

class Flatten : public Layer {
 public:
  virtual bool Load(const apollo::prediction::LayerParameter& layer_pb);
  virtual void Run(const std::vector<Eigen::MatrixXf>& inputs,
                   Eigen::MatrixXf* output);
};

class Input : public Layer {
 public:
  virtual bool Load(const apollo::prediction::LayerParameter& layer_pb);
  virtual void Run(const std::vector<Eigen::MatrixXf>& inputs,
                   Eigen::MatrixXf* output);

 private:
  std::vector<int> input_shape_;
  std::string dtype_;
  bool sparse_;
};

class Concatenate : public Layer {
 public:
  virtual bool Load(const apollo::prediction::LayerParameter& layer_pb);
  virtual void Run(const std::vector<Eigen::MatrixXf>& inputs,
                   Eigen::MatrixXf* output);

 private:
  int axis_;
};

}  // namespace network
}  // namespace prediction
}  // namespace apollo

#endif  // MODULES_PREDICTION_EVALUATOR_NETWORK_LAYER_LAYER_H_
