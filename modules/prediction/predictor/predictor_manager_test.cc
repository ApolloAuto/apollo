/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/prediction/predictor/predictor_manager.h"

#include <string>

#include "modules/prediction/proto/prediction_conf.pb.h"

#include "gtest/gtest.h"
#include "modules/common/util/file.h"

namespace apollo {
namespace prediction {

class PredictorManagerTest : public ::testing::Test {
 public:
  void SetUp() override { manager_ = PredictorManager::instance(); }

 protected:
  PredictorManager *manager_;
  PredictionConf conf_;
};

TEST_F(PredictorManagerTest, GetPredictor) {
  std::string conf_file = "modules/prediction/testdata/prediction_conf.pb.txt";
  CHECK(apollo::common::util::GetProtoFromFile(conf_file, &conf_))
      << "Failed to load " << conf_file;

  manager_->Init(conf_);

  const ObstacleConf::PredictorType type = ObstacleConf::FREE_MOVE_PREDICTOR;
  EXPECT_TRUE(manager_->GetPredictor(type) != nullptr);
}

}  // namespace prediction
}  // namespace apollo
