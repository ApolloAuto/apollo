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

#include "modules/prediction/evaluator/vehicle/mlp_evaluator.h"

#include <string>

#include "gtest/gtest.h"

#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/prediction/common/prediction_gflags.h"
#include "modules/common/util/file.h"

namespace apollo {
namespace prediction {

class MLPEvaluatorTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    std::string file =
        "modules/prediction/testdata/1_single_perception_vehicle.pb.txt";
    apollo::common::util::GetProtoFromFile(file, &perception_obstacles_);
  }
 protected:
  apollo::perception::PerceptionObstacles perception_obstacles_;
};

TEST_F(MLPEvaluatorTest, Constructor) {
  MLPEvaluator mlp_evaluator;
}

}  // namespace prediction
}  // namespace apollo
