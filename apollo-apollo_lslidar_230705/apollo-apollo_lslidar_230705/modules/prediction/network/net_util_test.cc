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

#include "gtest/gtest.h"

namespace apollo {
namespace prediction {
namespace network {

TEST(NetworkUtil, serialize_to_function_test) {
  std::function<float(float)> linear_func = serialize_to_function("linear");
  EXPECT_FLOAT_EQ(linear_func(-2.0), -2.0);
  EXPECT_FLOAT_EQ(linear_func(2.0), 2.0);

  std::function<float(float)> tanh_func = serialize_to_function("tanh");
  EXPECT_FLOAT_EQ(tanh_func(-2.0), -0.96402758);
  EXPECT_FLOAT_EQ(tanh_func(0.0), 0.0);
  EXPECT_FLOAT_EQ(tanh_func(2.0), 0.96402758);

  std::function<float(float)> sigm_func = serialize_to_function("sigmoid");
  EXPECT_FLOAT_EQ(sigm_func(-2.0), 0.11920292);
  EXPECT_FLOAT_EQ(sigm_func(0.0), 0.5);
  EXPECT_FLOAT_EQ(sigm_func(2.0), 0.88079707);

  std::function<float(float)> hsigm_func =
      serialize_to_function("hard_sigmoid");
  EXPECT_FLOAT_EQ(hsigm_func(-3.0), 0.0);
  EXPECT_FLOAT_EQ(hsigm_func(0.0), 0.5);
  EXPECT_FLOAT_EQ(hsigm_func(3.0), 1.0);

  std::function<float(float)> relu_func = serialize_to_function("relu");
  EXPECT_FLOAT_EQ(relu_func(-3.0), 0.0);
  EXPECT_FLOAT_EQ(relu_func(0.0), 0.0);
  EXPECT_FLOAT_EQ(relu_func(3.0), 3.0);
}

TEST(NetworkUtil, LoadTensor_test) {
  TensorParameter tensor_pb;
  Eigen::MatrixXf mat;
  Eigen::VectorXf vec;
  EXPECT_FALSE(LoadTensor(tensor_pb, &mat));
  EXPECT_FALSE(LoadTensor(tensor_pb, &vec));

  tensor_pb.add_shape(2);
  tensor_pb.add_data(1.0);
  tensor_pb.add_data(2.0);
  EXPECT_TRUE(LoadTensor(tensor_pb, &mat));
  EXPECT_FLOAT_EQ(mat(0, 0), 1.0);
  EXPECT_FLOAT_EQ(mat(0, 1), 2.0);

  EXPECT_TRUE(LoadTensor(tensor_pb, &vec));
  EXPECT_FLOAT_EQ(vec(0), 1.0);
  EXPECT_FLOAT_EQ(vec(1), 2.0);

  tensor_pb.add_shape(2);
  tensor_pb.add_data(3.0);
  tensor_pb.add_data(4.0);
  EXPECT_TRUE(LoadTensor(tensor_pb, &mat));
  EXPECT_FLOAT_EQ(mat(0, 0), 1.0);
  EXPECT_FLOAT_EQ(mat(0, 1), 2.0);
  EXPECT_FLOAT_EQ(mat(1, 0), 3.0);
  EXPECT_FLOAT_EQ(mat(1, 1), 4.0);
}

}  // namespace network
}  // namespace prediction
}  // namespace apollo
