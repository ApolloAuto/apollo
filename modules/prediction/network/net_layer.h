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

#include "modules/prediction/network/net_util.h"
#include "modules/prediction/proto/network_layers.pb.h"

#ifndef MODULES_PREDICTION_NETWORK_NET_LAYER_H_
#define MODULES_PREDICTION_NETWORK_NET_LAYER_H_

/**
 * @namespace apollo::prediction::network
 * @brief apollo::prediction::network
 */
namespace apollo {
namespace prediction {
namespace network {

/**
 * @class Layer
 * @brief Layer is a base class for specific network layers
 *        It contains a pure virtual function Run which must be implemeted
 *        in derived class
 */
class Layer {
 public:
  /**
   * @brief Constructor
   */
  Layer() = default;

  /**
   * @brief Destructor
   */
  virtual ~Layer() = default;

  /**
   * @brief Load layer parameters from a protobuf message
   * @param LayerParameter is a protobuf message
   * @return True if successly loaded, otherwise False
   */
  virtual bool Load(const apollo::prediction::LayerParameter& layer_pb);

  /**
   * @brief Reset the internal state of a layer such as LSTM, GRU
   */
  virtual void ResetState() {}

  /**
   * @brief Set the internal state of a layer
   * @param A specified internal state in a vector of Eigen::MatrixXf
   */
  virtual void SetState(const std::vector<Eigen::MatrixXf>& states) {}

  /**
   * @brief Access to the internal state of a layer
   * @return Internal state in a vector of Eigen::MatrixXf
   */
  virtual void State(std::vector<Eigen::MatrixXf>* states) const {}

  /**
   * @brief Compute the layer output from inputs
   * @param Inputs to a network layer
   * @param Output of a network layer will be returned
   */
  virtual void Run(const std::vector<Eigen::MatrixXf>& inputs,
                   Eigen::MatrixXf* output) = 0;

  /**
   * @brief Name of a layer
   * @reture Name of a layer
   */
  std::string Name() const { return name_; }

  /**
   * @brief Order number of a layer in a network
   * @reture Order numer of a layer in a network
   */
  int OrderNumber() const { return order_number_; }

 private:
  std::string name_;
  int order_number_;
};

/**
 * @class Dense
 * @brief Dense is the forward fully connected network layer.
 *        Dense layer output is y = f(x*w + b),
 *        where x is the input, w the weight, b the bias and f the activation
 *
 *        Parameter w and b can be loaded from pb message. if bias is
 *        not used, b = 0. f is linear function at default.
 */
class Dense : public Layer {
 public:
  /**
   * @brief Load the dense layer parameter from a pb message
   * @param A pb message contains the parameters
   * @return True is loaded successively, otherwise False
   */
  virtual bool Load(const apollo::prediction::LayerParameter& layer_pb);

  /**
   * @brief Compute the layer output from inputs
   * @param Inputs to a network layer
   * @param Output of a network layer will be returned
   */
  virtual void Run(const std::vector<Eigen::MatrixXf>& inputs,
                   Eigen::MatrixXf* output);

 private:
  int units_;
  bool use_bias_;
  Eigen::MatrixXf weights_;
  Eigen::VectorXf bias_;
  std::function<float(float)> kactivation_;
};

/**
 * @class Activation
 * @brief Activation is an activation network layer.
 *        Activation layer output is y = f(x),
 *        where x is the input, y the output and f the activation funtion
 *
 *        Parameter f can be loaded from pb message
 */
class Activation : public Layer {
 public:
  /**
   * @brief Load the parameter from a pb message
   * @param A pb message contains the parameters
   * @return True is loaded successively, otherwise False
   */
  virtual bool Load(const apollo::prediction::LayerParameter& layer_pb);

  /**
   * @brief Compute the layer output from inputs
   * @param Inputs to a network layer
   * @param Output of a network layer will be returned
   */
  virtual void Run(const std::vector<Eigen::MatrixXf>& inputs,
                   Eigen::MatrixXf* output);

 private:
  std::function<float(float)> kactivation_;
};

/**
 * @class Batch normalization layer (Ioffe and Szegedy, 2014)
 * @brief Normalize the previous layer. The layer output is
 *        y = (x - mu) / sqrt(sigma) * gamma + beta
 */
class BatchNormalization : public Layer {
 public:
  /**
   * @brief Load the parameter from a pb message
   * @param A pb message contains the parameters
   * @return True is loaded successively, otherwise False
   */
  virtual bool Load(const apollo::prediction::LayerParameter& layer_pb);

  /**
   * @brief Compute the layer output from inputs
   * @param Inputs to a network layer
   * @param Output of a network layer will be returned
   */
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

/**
 * @class LSTM, short of Long-Short Term Memory unit - Hochreiter 1997.
 * @brief For a step-by-step description of the algorithm, see
 *     [this tutorial](http://deeplearning.net/tutorial/lstm.html).
 */
class LSTM : public Layer {
 public:
  /**
   * @brief Load the layer parameter from a pb message
   * @param A pb message contains the parameters
   * @return True is loaded successively, otherwise False
   */
  bool Load(const apollo::prediction::LayerParameter& layer_pb);

  /**
   * @brief Compute the layer output from inputs
   * @param Inputs to a network layer
   * @param Output of a network layer will be returned
   */
  virtual void Run(const std::vector<Eigen::MatrixXf>& inputs,
                   Eigen::MatrixXf* output);

  /**
   * @brief Compute the output of lstm step by step
   * @param Input of current step
   * @param Ouput of current step
   * @parem Hidden state of previous step and return current hidden state
   * @param Cell state of previous step and return current cell state
   */
  void Step(const Eigen::MatrixXf& input, Eigen::MatrixXf* output,
            Eigen::MatrixXf* ht_1, Eigen::MatrixXf* ct_1);

  /**
   * @brief Reset the interal state and memery cell state as zero-matrix
   */
  virtual void ResetState();

  /**
   * @brief Set the internal state and memory cell state
   * @param A vector of Eigen::MatrixXf
   */
  virtual void SetState(const std::vector<Eigen::MatrixXf>& states);

  /**
   * @brief Access to the internal state and memory cell state
   * @return State in a vector of Eigen::MatrixXf
   */
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

/**
 * @class Flatten, It a network layer to flatten a matrix into vector.
 */
class Flatten : public Layer {
 public:
  /**
   * @brief Load the dense layer parameter from a pb message
   * @param A pb message contains the parameters
   * @return True is loaded successively, otherwise False
   */
  virtual bool Load(const apollo::prediction::LayerParameter& layer_pb);

  /**
   * @brief Compute the layer output from inputs
   * @param Inputs to a network layer
   * @param Output of a network layer will be returned
   */
  virtual void Run(const std::vector<Eigen::MatrixXf>& inputs,
                   Eigen::MatrixXf* output);
};

/**
 * @class Input, It is the input layer for a network, which specified
 *               the shape of input matrix
 */
class Input : public Layer {
 public:
  /**
   * @brief Load the layer parameter from a pb message
   * @param A pb message contains the parameters
   * @return True is loaded successively, otherwise False
   */
  virtual bool Load(const apollo::prediction::LayerParameter& layer_pb);

  /**
   * @brief Compute the layer output from inputs
   * @param Inputs to a network layer
   * @param Output of a network layer will be returned
   */
  virtual void Run(const std::vector<Eigen::MatrixXf>& inputs,
                   Eigen::MatrixXf* output);

 private:
  std::vector<int> input_shape_;
  std::string dtype_;
  bool sparse_;
};

/**
 * @class Concatenate, layer that concatenates a vector of inputs, return
 *                     a single matrix.
 */
class Concatenate : public Layer {
 public:
  /**
   * @brief Load the layer parameter from a pb message
   * @param A pb message contains the parameters
   * @return True is loaded successively, otherwise False
   */
  virtual bool Load(const apollo::prediction::LayerParameter& layer_pb);

  /**
   * @brief Compute the layer output from inputs
   * @param Inputs to a network layer
   * @param Output of a network layer will be returned
   */
  virtual void Run(const std::vector<Eigen::MatrixXf>& inputs,
                   Eigen::MatrixXf* output);

 private:
  int axis_;
};

}  // namespace network
}  // namespace prediction
}  // namespace apollo

#endif  // MODULES_PREDICTION_NETWORK_NET_LAYER_H_
