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

#include <unordered_map>

#include "cyber/common/log.h"

namespace apollo {
namespace prediction {
namespace network {

float sigmoid(const float x) { return 1.0f / (1.0f + std::exp(-x)); }

float tanh(const float x) { return std::tanh(x); }

float linear(const float x) { return x; }

float hard_sigmoid(const float x) {
  const float z = 0.2f * x + 0.5f;
  return z <= 0.0f ? 0.0f : (z <= 1.0f ? z : 1.0f);
}

float relu(const float x) { return (x > 0.0f) ? x : 0.0f; }

Eigen::MatrixXf FlattenMatrix(const Eigen::MatrixXf& matrix) {
  CHECK_GT(matrix.rows(), 0);
  CHECK_GT(matrix.cols(), 0);
  int output_size = static_cast<int>(matrix.rows() * matrix.cols());
  Eigen::MatrixXf output_matrix;
  output_matrix.resize(1, output_size);
  int output_index = 0;
  for (int i = 0; i < matrix.rows(); ++i) {
    for (int j = 0; j < matrix.cols(); ++j) {
      output_matrix(0, output_index) = matrix(i, j);
      ++output_index;
    }
  }
  return output_matrix;
}

std::function<float(float)> serialize_to_function(const std::string& str) {
  static const std::unordered_map<std::string, std::function<float(float)>>
      func_map({{"linear", linear},
                {"tanh", tanh},
                {"sigmoid", sigmoid},
                {"hard_sigmoid", hard_sigmoid},
                {"relu", relu}});
  return func_map.at(str);
}

bool LoadTensor(const TensorParameter& tensor_pb, Eigen::MatrixXf* matrix) {
  if (tensor_pb.data().empty() || tensor_pb.shape().empty()) {
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
  if (tensor_pb.data().empty() || tensor_pb.shape().empty()) {
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

bool LoadTensor(const TensorParameter& tensor_pb,
                std::vector<Eigen::MatrixXf>* const tensor3d) {
  if (tensor_pb.data().empty() || tensor_pb.shape_size() != 3) {
    AERROR << "Fail to load the necessary fields!";
    return false;
  }
  int num_depth = tensor_pb.shape(0);
  int num_row = tensor_pb.shape(1);
  int num_col = tensor_pb.shape(2);
  CHECK_EQ(tensor_pb.data_size(), num_depth * num_row * num_col);
  int tensor_pb_index = 0;
  for (int k = 0; k < num_depth; ++k) {
    Eigen::MatrixXf matrix = Eigen::MatrixXf::Zero(num_row, num_col);
    for (int i = 0; i < num_row; ++i) {
      for (int j = 0; j < num_col; ++j) {
        matrix(i, j) = tensor_pb.data(tensor_pb_index);
        ++tensor_pb_index;
      }
    }
    tensor3d->push_back(matrix);
  }
  CHECK_EQ(tensor_pb_index, num_depth * num_row * num_col);
  return true;
}

}  // namespace network
}  // namespace prediction
}  // namespace apollo
