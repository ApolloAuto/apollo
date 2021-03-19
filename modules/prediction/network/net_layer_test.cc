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

#include "modules/prediction/network/net_layer.h"

#include "gtest/gtest.h"

namespace apollo {
namespace prediction {
namespace network {

TEST(LayerTest, dense_test) {
  LayerParameter layer_pb;
  Dense dense;

  EXPECT_FALSE(dense.Load(layer_pb));
  EXPECT_EQ(dense.Name(), "layer");
  EXPECT_EQ(dense.OrderNumber(), -1);

  layer_pb.set_name("layer1");
  layer_pb.set_order_number(1);
  EXPECT_FALSE(dense.Load(layer_pb));
  EXPECT_EQ(dense.Name(), "layer1");
  EXPECT_EQ(dense.OrderNumber(), 1);

  layer_pb.mutable_dense()->set_units(1);
  layer_pb.mutable_dense()->mutable_weights()->add_shape(1);
  layer_pb.mutable_dense()->mutable_weights()->add_shape(1);
  layer_pb.mutable_dense()->mutable_weights()->add_data(1);
  layer_pb.mutable_dense()->mutable_bias()->add_shape(1);
  layer_pb.mutable_dense()->mutable_bias()->add_data(1);
  EXPECT_TRUE(dense.Load(layer_pb));
  layer_pb.mutable_dense()->set_activation("relu");
  EXPECT_TRUE(dense.Load(layer_pb));

  Eigen::MatrixXf output;
  Eigen::MatrixXf input(1, 1);
  input(0, 0) = 1.0;
  dense.Run({input}, &output);
  EXPECT_EQ(output(0, 0), 2.0);
}

TEST(LayerTest, activation_test) {
  LayerParameter layer_pb;
  Activation act;

  layer_pb.set_name("layer1");
  layer_pb.set_order_number(1);
  EXPECT_TRUE(act.Load(layer_pb));
  Eigen::MatrixXf output;
  Eigen::MatrixXf input(1, 1);
  input(0, 0) = -1.0;
  act.Run({input}, &output);
  EXPECT_EQ(output(0, 0), -1.0);

  layer_pb.mutable_activation()->set_activation("relu");
  EXPECT_TRUE(act.Load(layer_pb));
  act.Run({input}, &output);
  EXPECT_EQ(output(0, 0), 0.0);
}

TEST(LayerTest, bn_test) {
  LayerParameter layer_pb;
  BatchNormalization bn;

  layer_pb.mutable_batch_normalization()->set_epsilon(1e-8);
  layer_pb.mutable_batch_normalization()->set_axis(0);
  layer_pb.mutable_batch_normalization()->set_center(false);
  layer_pb.mutable_batch_normalization()->set_scale(false);
  layer_pb.mutable_batch_normalization()->set_momentum(2.0);
  layer_pb.mutable_batch_normalization()->mutable_mu()->add_shape(2);
  layer_pb.mutable_batch_normalization()->mutable_mu()->add_data(0.0);
  layer_pb.mutable_batch_normalization()->mutable_mu()->add_data(1.0);
  layer_pb.mutable_batch_normalization()->mutable_sigma()->add_shape(2);
  layer_pb.mutable_batch_normalization()->mutable_sigma()->add_data(1.0);
  layer_pb.mutable_batch_normalization()->mutable_sigma()->add_data(4.0);
  EXPECT_TRUE(bn.Load(layer_pb));

  Eigen::MatrixXf output;
  Eigen::MatrixXf input(2, 2);
  input(0, 0) = 1.0;
  input(1, 0) = 2.0;
  input(0, 1) = 3.0;
  input(1, 1) = 4.0;
  bn.Run({input}, &output);
  EXPECT_FLOAT_EQ(output(0, 0), 1.0);
  EXPECT_FLOAT_EQ(output(0, 1), 1.0);
  EXPECT_FLOAT_EQ(output(1, 0), 2.0);
  EXPECT_FLOAT_EQ(output(1, 1), 1.5);
}

TEST(LayerTest, lstm_test) {
  LayerParameter layer_pb;
  LSTM lstm;

  layer_pb.mutable_lstm()->set_units(1);
  layer_pb.mutable_lstm()->set_return_sequences(true);
  layer_pb.mutable_lstm()->set_stateful(true);
  layer_pb.mutable_lstm()->set_activation("sigmoid");
  layer_pb.mutable_lstm()->set_recurrent_activation("hard_sigmoid");
  layer_pb.mutable_lstm()->set_use_bias(true);
  layer_pb.mutable_lstm()->set_unit_forget_bias(false);
  layer_pb.mutable_lstm()->mutable_weights_input()->add_shape(1);
  layer_pb.mutable_lstm()->mutable_weights_input()->add_shape(1);
  layer_pb.mutable_lstm()->mutable_weights_input()->add_data(1);
  layer_pb.mutable_lstm()->mutable_weights_forget()->add_shape(1);
  layer_pb.mutable_lstm()->mutable_weights_forget()->add_shape(1);
  layer_pb.mutable_lstm()->mutable_weights_forget()->add_data(1);
  layer_pb.mutable_lstm()->mutable_weights_cell()->add_shape(1);
  layer_pb.mutable_lstm()->mutable_weights_cell()->add_shape(1);
  layer_pb.mutable_lstm()->mutable_weights_cell()->add_data(1);
  layer_pb.mutable_lstm()->mutable_weights_output()->add_shape(1);
  layer_pb.mutable_lstm()->mutable_weights_output()->add_shape(1);
  layer_pb.mutable_lstm()->mutable_weights_output()->add_data(1);
  layer_pb.mutable_lstm()->mutable_bias_input()->add_shape(1);
  layer_pb.mutable_lstm()->mutable_bias_input()->add_data(1);
  layer_pb.mutable_lstm()->mutable_bias_forget()->add_shape(1);
  layer_pb.mutable_lstm()->mutable_bias_forget()->add_data(1);
  layer_pb.mutable_lstm()->mutable_bias_cell()->add_shape(1);
  layer_pb.mutable_lstm()->mutable_bias_cell()->add_data(1);
  layer_pb.mutable_lstm()->mutable_bias_output()->add_shape(1);
  layer_pb.mutable_lstm()->mutable_bias_output()->add_data(1);
  layer_pb.mutable_lstm()->mutable_recurrent_weights_input()->add_shape(1);
  layer_pb.mutable_lstm()->mutable_recurrent_weights_input()->add_shape(1);
  layer_pb.mutable_lstm()->mutable_recurrent_weights_input()->add_data(1);
  layer_pb.mutable_lstm()->mutable_recurrent_weights_forget()->add_shape(1);
  layer_pb.mutable_lstm()->mutable_recurrent_weights_forget()->add_shape(1);
  layer_pb.mutable_lstm()->mutable_recurrent_weights_forget()->add_data(1);
  layer_pb.mutable_lstm()->mutable_recurrent_weights_cell()->add_shape(1);
  layer_pb.mutable_lstm()->mutable_recurrent_weights_cell()->add_shape(1);
  layer_pb.mutable_lstm()->mutable_recurrent_weights_cell()->add_data(1);
  EXPECT_FALSE(lstm.Load(layer_pb));
  layer_pb.mutable_lstm()->mutable_recurrent_weights_output()->add_shape(1);
  layer_pb.mutable_lstm()->mutable_recurrent_weights_output()->add_shape(1);
  layer_pb.mutable_lstm()->mutable_recurrent_weights_output()->add_data(1);
  EXPECT_TRUE(lstm.Load(layer_pb));

  std::vector<Eigen::MatrixXf> state;
  lstm.ResetState();
  lstm.State(&state);
  EXPECT_EQ(state.size(), 2);
  EXPECT_EQ(state[0](0, 0), 0);
  EXPECT_EQ(state[1](0, 0), 0);
  state[0](0, 0) = 1;
  state[1](0, 0) = 2;
  lstm.SetState(state);
  lstm.State(&state);
  EXPECT_EQ(state.size(), 2);
  EXPECT_EQ(state[0](0, 0), 1);
  EXPECT_EQ(state[1](0, 0), 2);
}

TEST(LayerTest, flatten_test) {
  LayerParameter layer_pb;
  Flatten flat;

  layer_pb.set_name("layer1");
  layer_pb.set_order_number(1);
  EXPECT_TRUE(flat.Load(layer_pb));

  Eigen::MatrixXf input(2, 2);
  input(0, 0) = 1.0;
  input(1, 0) = 2.0;
  input(0, 1) = 3.0;
  input(1, 1) = 4.0;
  Eigen::MatrixXf output;
  flat.Run({input}, &output);
  EXPECT_FLOAT_EQ(output(0, 0), 1.0);
  EXPECT_FLOAT_EQ(output(0, 1), 3.0);
  EXPECT_FLOAT_EQ(output(0, 2), 2.0);
  EXPECT_FLOAT_EQ(output(0, 3), 4.0);
}

TEST(LayerTest, input_test) {
  LayerParameter layer_pb;
  Input input;

  EXPECT_FALSE(input.Load(layer_pb));
  layer_pb.mutable_input()->add_input_shape(2);
  EXPECT_TRUE(input.Load(layer_pb));
}

TEST(LayerTest, concat_test) {
  LayerParameter layer_pb;
  Concatenate concat;

  layer_pb.mutable_concatenate()->set_axis(0);
  EXPECT_TRUE(concat.Load(layer_pb));

  Eigen::MatrixXf input1(2, 1);
  input1(0, 0) = 1.0;
  input1(1, 0) = 2.0;
  Eigen::MatrixXf input2(2, 1);
  input2(0, 0) = 3.0;
  input2(1, 0) = 4.0;

  Eigen::MatrixXf output;
  concat.Run({input1, input2}, &output);
  EXPECT_FLOAT_EQ(output(0, 0), 1.0);
  EXPECT_FLOAT_EQ(output(0, 1), 3.0);
  EXPECT_FLOAT_EQ(output(1, 0), 2.0);
  EXPECT_FLOAT_EQ(output(1, 1), 4.0);
}

}  // namespace network
}  // namespace prediction
}  // namespace apollo
