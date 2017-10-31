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
 * @file
 * @brief
 */

#ifndef MODULES_PREDICTION_EVALUATOR_NETWORK_LAYRE_UTIL_H_
#define MODULES_PREDICTION_EVALUATOR_NETWORK_LAYER_UTIL_H_

#include <functional>
#include <string>

#include "Eigen/Dense"

#include "modules/prediction/proto/network_layers.pb.h"

/**
 * @namespace apollo::prediction::network
 * @brief apollo::prediction::network
 */
namespace apollo {
namespace prediction {
namespace network {

float sigmoid(float x);
float tanh(float x);
float linear(float x);
float hard_sigmoid(float x);
float relu(float x);

std::function<float(float)> serialize_to_function(const std::string& str);

bool LoadTensor(const TensorParameter& tensor_pb, Eigen::MatrixXf* matrix);

bool LoadTensor(const TensorParameter& tensor_pb, Eigen::VectorXf* vector);

}  // namespace network
}  // namespace prediction
}  // namespace apollo

#endif  // MODULES_PREDICTION_EVALUATOR_NETWORK_LAYER_UTIL_H_
