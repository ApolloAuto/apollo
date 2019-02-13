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

#include "gtest/gtest.h"

#include "cyber/common/file.h"
#include "modules/prediction/network/rnn_model/rnn_model.h"

namespace apollo {
namespace prediction {
namespace network {

class NetModelTest : public ::testing::Test {
 public:
  void SetUp() override {}
};

TEST(NetModelTest, verification_test) {
  const std::string rnn_filename =
      "modules/prediction/data/rnn_vehicle_model.bin";
  NetParameter net_parameter = NetParameter();
  EXPECT_TRUE(cyber::common::GetProtoFromFile(rnn_filename, &net_parameter));
  EXPECT_TRUE(RnnModel::Instance()->LoadModel(net_parameter));

  Eigen::MatrixXf obstacle_feature;
  Eigen::MatrixXf lane_feature;
  Eigen::MatrixXf output;
  for (int i = 0; i < net_parameter.verification_samples_size(); ++i) {
    VerificationSample sample = net_parameter.verification_samples(i);
    EXPECT_EQ(sample.features_size(), 2);
    EXPECT_TRUE(LoadTensor(sample.features(0), &obstacle_feature));
    EXPECT_TRUE(LoadTensor(sample.features(1), &lane_feature));

    RnnModel::Instance()->Run({obstacle_feature, lane_feature}, &output);
    EXPECT_EQ(output.size(), 2);
    EXPECT_TRUE(sample.has_probability());
    EXPECT_NEAR(output(0, 0), sample.probability(), 0.1);

    RnnModel::Instance()->ResetState();
  }
}

}  // namespace network
}  // namespace prediction
}  // namespace apollo
