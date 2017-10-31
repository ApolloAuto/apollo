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

#include "modules/common/util/file.h"
#include "modules/prediction/evaluator/network/rnn_model.h"

namespace apollo {
namespace prediction {
namespace network {

class NetModelTest : public ::testing::Test {
 public:
  void SetUp() override {}
};

TEST(NetModelTest, verification_test) {
    const char* rnn_filename = "modules/prediction/data/rnn_vehicle_model.bin";
    NetParameter net_parameter = NetParameter();
    EXPECT_TRUE(common::util::GetProtoFromFile(rnn_filename, &net_parameter));
    EXPECT_TRUE(RnnModel::instance()->LoadModel(net_parameter));
    EXPECT_TRUE(RnnModel::instance()->VerifyModel());
}

}  // namespace network
}  // namespace prediction
}  // namespace apollo
