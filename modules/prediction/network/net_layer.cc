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

/**
 * @file layer.cc
 */

#include "modules/prediction/network/net_layer.h"

#include <algorithm>
#include <limits>

#include "cyber/common/log.h"

namespace apollo {
namespace prediction {
namespace network {

bool Layer::Load(const LayerParameter& layer_pb) {
  if (!layer_pb.has_name()) {
    ADEBUG << "Set name at default";
    name_ = "layer";
  } else {
    name_ = layer_pb.name();
  }
  if (!layer_pb.has_order_number()) {
    ADEBUG << "Set order number at default";
    order_number_ = -1;
  } else {
    order_number_ = layer_pb.order_number();
  }
  return true;
}

bool Dense::Load(const LayerParameter& layer_pb) {
  if (!Layer::Load(layer_pb)) {
    AERROR << "Fail to Load LayerParameter!";
    return false;
  }
  DenseParameter dense_pb = layer_pb.dense();
  return Load(dense_pb);
}

bool Dense::Load(const DenseParameter& dense_pb) {
  if (!dense_pb.has_weights() || !LoadTensor(dense_pb.weights(), &weights_)) {
    AERROR << "Fail to Load weights!";
    return false;
  }
  if (!dense_pb.has_bias() || !LoadTensor(dense_pb.bias(), &bias_)) {
    AERROR << "Fail to Load bias!";
    return false;
  }
  if (!dense_pb.has_use_bias()) {
    AWARN << "Set use_bias as false.";
    use_bias_ = true;
  } else {
    use_bias_ = dense_pb.use_bias();
  }
  if (!dense_pb.has_activation()) {
    ADEBUG << "Set activation as linear function";
    kactivation_ = serialize_to_function("linear");
  } else {
    kactivation_ = serialize_to_function(dense_pb.activation());
  }
  units_ = dense_pb.units();
  return true;
}

void Dense::Run(const std::vector<Eigen::MatrixXf>& inputs,
                Eigen::MatrixXf* output) {
  CHECK_EQ(inputs.size(), 1U);
  Eigen::MatrixXf prod = static_cast<Eigen::MatrixXf>(inputs[0] * weights_);
  if (use_bias_) {
    Eigen::MatrixXf sum = prod.rowwise() + bias_.transpose();
    prod = sum;
  }
  *output = prod.unaryExpr(kactivation_);
  CHECK_EQ(output->cols(), units_);
}

bool Conv1d::Load(const LayerParameter& layer_pb) {
  if (!Layer::Load(layer_pb)) {
    AERROR << "Fail to Load LayerParameter!";
    return false;
  }
  Conv1dParameter conv1d_pb = layer_pb.conv1d();
  return Load(conv1d_pb);
}

bool Conv1d::Load(const Conv1dParameter& conv1d_pb) {
  if (!conv1d_pb.has_kernel() || !LoadTensor(conv1d_pb.kernel(), &kernel_)) {
    AERROR << "Fail to Load kernel!";
    return false;
  }
  if (!conv1d_pb.has_bias() || !LoadTensor(conv1d_pb.bias(), &bias_)) {
    AERROR << "Fail to Load bias!";
    return false;
  }
  if (!conv1d_pb.has_use_bias()) {
    AWARN << "Set use_bias as false.";
    use_bias_ = true;
  } else {
    use_bias_ = conv1d_pb.use_bias();
  }
  for (int sz : conv1d_pb.shape()) {
    shape_.push_back(sz);
  }
  if (conv1d_pb.has_stride()) {
    stride_ = conv1d_pb.stride();
  } else {
    stride_ = 1;
  }
  return true;
}

void Conv1d::Run(const std::vector<Eigen::MatrixXf>& inputs,
                 Eigen::MatrixXf* output) {
  CHECK_EQ(inputs.size(), 1U);
  CHECK_GT(kernel_.size(), 0U);
  CHECK_EQ(kernel_[0].rows(), inputs[0].rows());
  int kernel_size = static_cast<int>(kernel_[0].cols());
  int output_num_col =
      static_cast<int>((inputs[0].cols() - kernel_size) / stride_) + 1;
  int output_num_row = static_cast<int>(kernel_.size());
  output->resize(output_num_row, output_num_col);
  for (int i = 0; i < output_num_row; ++i) {
    for (int j = 0; j < output_num_col; ++j) {
      float output_i_j_unbiased = 0.0f;
      for (int p = 0; p < inputs[0].rows(); ++p) {
        for (int q = j * stride_; q < j * stride_ + kernel_size; ++q) {
          output_i_j_unbiased +=
              inputs[0](p, q) * kernel_[i](p, q - j * stride_);
        }
      }

      (*output)(i, j) = output_i_j_unbiased + bias_(i);
    }
  }
}

bool MaxPool1d::Load(const LayerParameter& layer_pb) {
  if (!Layer::Load(layer_pb)) {
    AERROR << "Fail to Load LayerParameter!";
    return false;
  }
  MaxPool1dParameter maxpool1d_pb = layer_pb.maxpool1d();
  return Load(maxpool1d_pb);
}

bool MaxPool1d::Load(const MaxPool1dParameter& maxpool1d_pb) {
  ACHECK(maxpool1d_pb.has_kernel_size());
  CHECK_GT(maxpool1d_pb.has_kernel_size(), 0);
  kernel_size_ = maxpool1d_pb.kernel_size();
  if (maxpool1d_pb.has_stride() && maxpool1d_pb.stride() > 0) {
    stride_ = maxpool1d_pb.stride();
  } else {
    ADEBUG << "No valid stride found, use kernel size, instead";
    stride_ = maxpool1d_pb.kernel_size();
  }
  return true;
}

void MaxPool1d::Run(const std::vector<Eigen::MatrixXf>& inputs,
                    Eigen::MatrixXf* output) {
  CHECK_EQ(inputs.size(), 1U);
  int output_num_col =
      static_cast<int>((inputs[0].cols() - kernel_size_) / stride_) + 1;
  int output_num_row = static_cast<int>(inputs[0].rows());
  output->resize(output_num_row, output_num_col);
  int input_index = 0;
  for (int j = 0; j < output_num_col; ++j) {
    CHECK_LE(input_index + kernel_size_, inputs[0].cols());
    for (int i = 0; i < output_num_row; ++i) {
      float output_i_j = -std::numeric_limits<float>::infinity();
      for (int k = input_index; k < input_index + kernel_size_; ++k) {
        output_i_j = std::max(output_i_j, inputs[0](i, k));
      }
      (*output)(i, j) = output_i_j;
    }
    input_index += stride_;
  }
}

bool AvgPool1d::Load(const LayerParameter& layer_pb) {
  if (!Layer::Load(layer_pb)) {
    AERROR << "Fail to Load LayerParameter!";
    return false;
  }
  AvgPool1dParameter avgpool1d_pb = layer_pb.avgpool1d();
  return Load(avgpool1d_pb);
}

bool AvgPool1d::Load(const AvgPool1dParameter& avgpool1d_pb) {
  ACHECK(avgpool1d_pb.has_kernel_size());
  CHECK_GT(avgpool1d_pb.has_kernel_size(), 0);
  kernel_size_ = avgpool1d_pb.kernel_size();
  if (avgpool1d_pb.has_stride() && avgpool1d_pb.stride() > 0) {
    stride_ = avgpool1d_pb.stride();
  } else {
    ADEBUG << "No valid stride found, use kernel size, instead";
    stride_ = avgpool1d_pb.kernel_size();
  }
  return true;
}

void AvgPool1d::Run(const std::vector<Eigen::MatrixXf>& inputs,
                    Eigen::MatrixXf* output) {
  CHECK_EQ(inputs.size(), 1U);
  int output_num_col =
      static_cast<int>((inputs[0].cols() - kernel_size_) / stride_) + 1;
  int output_num_row = static_cast<int>(inputs[0].rows());
  output->resize(output_num_row, output_num_col);
  int input_index = 0;
  for (int j = 0; j < output_num_col; ++j) {
    CHECK_LE(input_index + kernel_size_, inputs[0].cols());
    for (int i = 0; i < output_num_row; ++i) {
      float output_i_j_sum = 0.0f;
      for (int k = input_index; k < input_index + kernel_size_; ++k) {
        output_i_j_sum += inputs[0](i, k);
      }
      (*output)(i, j) = output_i_j_sum / static_cast<float>(kernel_size_);
    }
    input_index += stride_;
  }
}

bool Activation::Load(const LayerParameter& layer_pb) {
  if (!Layer::Load(layer_pb)) {
    AERROR << "Fail to Load the layer parameters!";
    return false;
  }
  if (!layer_pb.has_activation()) {
    kactivation_ = serialize_to_function("linear");
  } else {
    ActivationParameter activation_pb = layer_pb.activation();
    kactivation_ = serialize_to_function(activation_pb.activation());
  }
  return true;
}

bool Activation::Load(const ActivationParameter& activation_pb) {
  if (!activation_pb.has_activation()) {
    kactivation_ = serialize_to_function("linear");
  } else {
    kactivation_ = serialize_to_function(activation_pb.activation());
  }
  return true;
}

void Activation::Run(const std::vector<Eigen::MatrixXf>& inputs,
                     Eigen::MatrixXf* output) {
  CHECK_EQ(inputs.size(), 1U);
  *output = inputs[0].unaryExpr(kactivation_);
}

bool BatchNormalization::Load(const LayerParameter& layer_pb) {
  if (!Layer::Load(layer_pb)) {
    AERROR << "Fail to Load the layer parameters!";
    return false;
  }

  auto bn_pb = layer_pb.batch_normalization();
  epsilon_ = static_cast<float>(bn_pb.epsilon());
  axis_ = bn_pb.axis();
  center_ = bn_pb.center();
  scale_ = bn_pb.scale();
  momentum_ = bn_pb.momentum();
  if (!bn_pb.has_mu() || !LoadTensor(bn_pb.mu(), &mu_)) {
    AERROR << "Fail to Load mu!";
    return false;
  }
  if (!bn_pb.has_sigma() || !LoadTensor(bn_pb.sigma(), &sigma_)) {
    AERROR << "Fail to Load sigma!";
    return false;
  }
  if (scale_) {
    if (!bn_pb.has_gamma() || !LoadTensor(bn_pb.gamma(), &gamma_)) {
      AERROR << "Fail to Load gamma!";
      return false;
    }
  }
  if (center_) {
    if (!bn_pb.has_beta() || !LoadTensor(bn_pb.beta(), &beta_)) {
      AERROR << "Fail to Load beta!";
      return false;
    }
  }
  return true;
}

void BatchNormalization::Run(const std::vector<Eigen::MatrixXf>& inputs,
                             Eigen::MatrixXf* output) {
  CHECK_EQ(inputs.size(), 1U);
  Eigen::MatrixXf temp = (inputs[0].rowwise() - mu_.transpose());
  Eigen::MatrixXf norm =
      temp.array().rowwise() / (sigma_.array().sqrt() + epsilon_).transpose();
  if (scale_) {
    norm = norm.array().rowwise() * gamma_.transpose().array();
  }
  if (center_) {
    norm = norm.rowwise() + beta_.transpose();
  }
  *output = norm;
}

bool LSTM::Load(const LayerParameter& layer_pb) {
  if (!Layer::Load(layer_pb)) {
    AERROR << "Fail to Load the layer parameters!";
    return false;
  }
  LSTMParameter lstm_pb = layer_pb.lstm();
  if (!lstm_pb.has_units()) {
    ADEBUG << "Fail to Load the number of units.";
    return false;
  } else {
    units_ = lstm_pb.units();
  }
  if (!lstm_pb.has_return_sequences()) {
    ADEBUG << "Set return_sequences at default.";
    return_sequences_ = false;
  } else {
    return_sequences_ = lstm_pb.return_sequences();
  }
  if (!lstm_pb.has_stateful()) {
    ADEBUG << "Set stateful at default.";
    stateful_ = false;
  } else {
    stateful_ = lstm_pb.stateful();
  }
  if (!lstm_pb.has_activation()) {
    ADEBUG << "Set activation function as tanh.";
    kactivation_ = serialize_to_function("tanh");
  } else {
    kactivation_ = serialize_to_function(lstm_pb.activation());
  }
  if (!lstm_pb.has_recurrent_activation()) {
    ADEBUG << "Set recurrent_activation function as hard_tanh.";
    krecurrent_activation_ = serialize_to_function("hard_tanh");
  } else {
    krecurrent_activation_ =
        serialize_to_function(lstm_pb.recurrent_activation());
  }
  if (!lstm_pb.has_use_bias()) {
    ADEBUG << "Set use_bias as true.";
    use_bias_ = true;
  } else {
    use_bias_ = lstm_pb.use_bias();
  }
  if (!lstm_pb.has_unit_forget_bias()) {
    ADEBUG << "Set unit forget bias as true.";
    unit_forget_bias_ = true;
  } else {
    unit_forget_bias_ = lstm_pb.unit_forget_bias();
  }
  if (!lstm_pb.has_weights_input() ||
      !LoadTensor(lstm_pb.weights_input(), &wi_)) {
    AERROR << "Fail to Load input weights!";
    return false;
  }
  if (!lstm_pb.has_weights_forget() ||
      !LoadTensor(lstm_pb.weights_forget(), &wf_)) {
    AERROR << "Fail to Load forget weights!";
    return false;
  }
  if (!lstm_pb.has_weights_cell() ||
      !LoadTensor(lstm_pb.weights_cell(), &wc_)) {
    AERROR << "Fail to Load cell weights!";
    return false;
  }
  if (!lstm_pb.has_weights_output() ||
      !LoadTensor(lstm_pb.weights_output(), &wo_)) {
    AERROR << "Fail to Load output weights!";
    return false;
  }
  if (!lstm_pb.has_bias_input() || !LoadTensor(lstm_pb.bias_input(), &bi_)) {
    AERROR << "Fail to Load input bias!";
    return false;
  }
  if (!lstm_pb.has_bias_forget() || !LoadTensor(lstm_pb.bias_forget(), &bf_)) {
    AERROR << "Fail to Load forget bias!";
    return false;
  }
  if (!lstm_pb.has_bias_cell() || !LoadTensor(lstm_pb.bias_cell(), &bc_)) {
    AERROR << "Fail to Load cell bias!";
    return false;
  }
  if (!lstm_pb.has_bias_output() || !LoadTensor(lstm_pb.bias_output(), &bo_)) {
    AERROR << "Fail to Load output bias!";
    return false;
  }
  if (!lstm_pb.has_recurrent_weights_input() ||
      !LoadTensor(lstm_pb.recurrent_weights_input(), &r_wi_)) {
    AERROR << "Fail to Load recurrent input weights!";
    return false;
  }
  if (!lstm_pb.has_recurrent_weights_forget() ||
      !LoadTensor(lstm_pb.recurrent_weights_forget(), &r_wf_)) {
    AERROR << "Fail to Load recurrent forget weights!";
    return false;
  }
  if (!lstm_pb.has_recurrent_weights_cell() ||
      !LoadTensor(lstm_pb.recurrent_weights_cell(), &r_wc_)) {
    AERROR << "Fail to Load recurrent cell weights!";
    return false;
  }
  if (!lstm_pb.has_recurrent_weights_output() ||
      !LoadTensor(lstm_pb.recurrent_weights_output(), &r_wo_)) {
    AERROR << "Fail to Load recurrent output weights!";
    return false;
  }
  ResetState();
  return true;
}

void LSTM::Step(const Eigen::MatrixXf& input, Eigen::MatrixXf* output,
                Eigen::MatrixXf* ht_1, Eigen::MatrixXf* ct_1) {
  Eigen::MatrixXf x_i = input * wi_ + bi_.transpose();
  Eigen::MatrixXf x_f = input * wf_ + bf_.transpose();
  Eigen::MatrixXf x_c = input * wc_ + bc_.transpose();
  Eigen::MatrixXf x_o = input * wo_ + bo_.transpose();

  Eigen::MatrixXf i = (x_i + (*ht_1) * r_wi_).unaryExpr(krecurrent_activation_);
  Eigen::MatrixXf f = (x_f + (*ht_1) * r_wf_).unaryExpr(krecurrent_activation_);
  Eigen::MatrixXf c =
      f.array() * ct_1->array() +
      i.array() * ((x_c + (*ht_1) * r_wc_).unaryExpr(kactivation_)).array();
  Eigen::MatrixXf o = (x_o + (*ht_1) * r_wo_).unaryExpr(krecurrent_activation_);
  Eigen::MatrixXf h = o.array() * (c.unaryExpr(kactivation_)).array();

  *ht_1 = h;
  *ct_1 = c;
  *output = h;
}

void LSTM::Run(const std::vector<Eigen::MatrixXf>& inputs,
               Eigen::MatrixXf* output) {
  CHECK_EQ(inputs.size(), 1U);
  Eigen::MatrixXf sequences(inputs[0].rows(), units_);
  Eigen::MatrixXf temp;
  for (int i = 0; i < inputs[0].rows(); ++i) {
    Step(inputs[0].row(i), &temp, &ht_1_, &ct_1_);
    sequences.row(i) = temp.row(0);
  }
  if (return_sequences_) {
    *output = sequences;
  } else {
    *output = temp.row(0);
  }
}

void LSTM::ResetState() {
  ht_1_.resize(1, units_);
  ct_1_.resize(1, units_);
  ht_1_.fill(0.0);
  ct_1_.fill(0.0);
}

void LSTM::State(std::vector<Eigen::MatrixXf>* states) const {
  states->resize(2);
  states->at(0) = ht_1_;
  states->at(1) = ct_1_;
}

void LSTM::SetState(const std::vector<Eigen::MatrixXf>& states) {
  CHECK_EQ(states.size(), 2U);
  CHECK_EQ(states[0].rows(), 1);
  CHECK_EQ(states[1].rows(), 1);
  CHECK_EQ(states[0].cols(), units_);
  CHECK_EQ(states[1].cols(), units_);
  ht_1_ = states[0];
  ct_1_ = states[1];
}

bool Flatten::Load(const LayerParameter& layer_pb) {
  if (!Layer::Load(layer_pb)) {
    AERROR << "Fail to Load the layer parameters!";
    return false;
  }
  return true;
}

void Flatten::Run(const std::vector<Eigen::MatrixXf>& inputs,
                  Eigen::MatrixXf* output) {
  CHECK_EQ(inputs.size(), 1U);
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> inp(
      inputs[0]);
  inp.resize(1, inp.size());
  *output = inp;
}

bool Input::Load(const LayerParameter& layer_pb) {
  if (!Layer::Load(layer_pb)) {
    AERROR << "Fail to Load the layer parameters!";
    return false;
  }
  InputParameter input_pb = layer_pb.input();
  if (input_pb.input_shape_size() < 1) {
    AERROR << "Fail to Load input shape of InputLayer!";
    return false;
  } else {
    input_shape_.resize(input_pb.input_shape_size());
    for (int i = 0; i < input_pb.input_shape_size(); ++i) {
      input_shape_[i] = input_pb.input_shape(i);
    }
  }
  if (!input_pb.has_dtype()) {
    ADEBUG << "Set the type of input as float!";
    dtype_ = "float32";
  } else {
    dtype_ = input_pb.dtype();
  }
  if (!input_pb.has_sparse()) {
    ADEBUG << "set the sparse of input as false!";
    sparse_ = false;
  } else {
    sparse_ = input_pb.sparse();
  }
  return true;
}

void Input::Run(const std::vector<Eigen::MatrixXf>& inputs,
                Eigen::MatrixXf* output) {
  CHECK_EQ(inputs.size(), 1U);
  CHECK_EQ(inputs[0].cols(), input_shape_.back());
  *output = inputs[0];
}

bool Concatenate::Load(const LayerParameter& layer_pb) {
  if (!Layer::Load(layer_pb)) {
    AERROR << "Fail to Load the layer parameters!";
    return false;
  }
  ConcatenateParameter concat_pb = layer_pb.concatenate();
  if (!concat_pb.has_axis()) {
    AERROR << "Fail to Load the concatenate!";
    return false;
  }
  axis_ = concat_pb.axis();
  return true;
}

void Concatenate::Run(const std::vector<Eigen::MatrixXf>& inputs,
                      Eigen::MatrixXf* output) {
  CHECK_EQ(inputs.size(), 2U);
  CHECK_EQ(inputs[0].rows(), inputs[1].rows());
  output->resize(inputs[0].rows(), inputs[0].cols() + inputs[1].cols());
  *output << inputs[0], inputs[1];
}

}  // namespace network
}  // namespace prediction
}  // namespace apollo
