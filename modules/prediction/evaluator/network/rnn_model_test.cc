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

#include "modules/prediction/evaluator/network/rnn_model.h"

namespace apollo {
namespace prediction {
namespace network {

const char* rnn_model_filename = "../../data/rnn_vehicle_model.bin";

TEST(RnnModel, verification_test) {
    RnnModel::instance()->LoadFromProtobuf(rnn_model_filename);
    EXPECT_TRUE(RnnModel::instance()->VerifyModel());
}

}  // namespace network
}  // namespace prediction
}  // namespace apollo
