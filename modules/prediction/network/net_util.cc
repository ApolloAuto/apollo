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

#include "modules/prediction/network/net_util.h"

#include <string>
#include <unordered_map>
#include "Eigen/Dense"

#include "modules/common/log.h"

namespace apollo {
namespace prediction {
namespace network {

float sigmoid(float x) { return 1 / (1 + exp(-1 * x)); }

float tanh(float x) { return std::tanh(x); }

float linear(float x) { return x; }

float hard_sigmoid(float x) {
  float z = 0.2 * x + 0.5;
  return z <= 0.0 ? 0.0 : (z <= 1.0 ? z : 1.0);
}

float relu(float x) { return (x > 0.0) ? x : 0.0; }

std::function<float(float)> serialize_to_function(const std::string& str) {
  static const std::unordered_map<std::string, std::function<float(float)> >
      func_map({{"linear", linear},
                {"tanh", tanh},
                {"sigmoid", sigmoid},
                {"hard_sigmoid", hard_sigmoid},
                {"relu", relu}});
  return func_map.at(str);
}

bool LoadTensor(const TensorParameter& tensor_pb, Eigen::MatrixXf* matrix) {
  if (tensor_pb.data_size() == 0 || tensor_pb.shape_size() == 0) {
    AERROR << "Fail to load the necessary fields!";
    return false;
  }
  if (tensor_pb.shape_size() < 2) {
    ADEBUG << "Load tensor size: (1, " << tensor_pb.shape(0) << ")";
    matrix->resize(1, tensor_pb.shape(0));
    for (int i = 0; i < tensor_pb.shape(0); ++i) {
      (*matrix)(0, i) = static_cast<float>(tensor_pb.data(i));
    }
    return true;
  }
  ADEBUG << "Load tensor size: (" << tensor_pb.shape(0) << ", "
         << tensor_pb.shape(1) << ")";
  CHECK_EQ(tensor_pb.shape_size(), 2);
  matrix->resize(tensor_pb.shape(0), tensor_pb.shape(1));
  for (int i = 0; i < tensor_pb.shape(0); ++i) {
    for (int j = 0; j < tensor_pb.shape(1); ++j) {
      (*matrix)(i, j) =
          static_cast<float>(tensor_pb.data(i * tensor_pb.shape(1) + j));
    }
  }
  return true;
}

bool LoadTensor(const TensorParameter& tensor_pb, Eigen::VectorXf* vector) {
  if (tensor_pb.data_size() == 0 || tensor_pb.shape_size() == 0) {
    AERROR << "Fail to load the necessary fields!";
    return false;
  }
  ADEBUG << "Load tensor size: (" << tensor_pb.shape(0) << ", 1)";
  CHECK_EQ(tensor_pb.shape_size(), 1);
  if (tensor_pb.shape_size() == 1) {
    vector->resize(tensor_pb.shape(0));
    for (int i = 0; i < tensor_pb.shape(0); ++i) {
      (*vector)(i) = static_cast<float>(tensor_pb.data(i));
    }
  }
  return true;
}

}  // namespace network
}  // namespace prediction
}  // namespace apollo
