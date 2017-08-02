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

#include "modules/prediction/predictor/vehicle/free_move_predictor.h"

#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"
#include "modules/prediction/common/prediction_gflags.h"
#include "modules/common/util/file.h"
#include "modules/prediction/container/obstacles/obstacle.h"
#include "modules/prediction/container/obstacles/obstacles_container.h"

namespace apollo {
namespace prediction {

class FreeMovePredictorTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    std::string file =
      "modules/prediction/testdata/single_perception_vehicle_offlane.pb.txt";
    apollo::common::util::GetProtoFromFile(file, &perception_obstacles_);
    FLAGS_map_file = "modules/prediction/testdata/kml_map.bin";
  }
 protected:
  apollo::perception::PerceptionObstacles perception_obstacles_;
};

TEST_F(FreeMovePredictorTest, General) {
}

}  // namespace prediction
}  // namespace apollo
