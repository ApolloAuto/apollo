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

#include <string>

#include "gtest/gtest.h"

#include "modules/prediction/predictor/predictor_manager.h"
#include "modules/prediction/proto/prediction_conf.pb.h"
#include "modules/common/util/file.h"

namespace apollo {
namespace prediction {

class PredictorManagerTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    manager_ = PredictorManager::instance();
  }

 protected:
  PredictorManager *manager_;
  PredictionConf conf_;
};

TEST_F(PredictorManagerTest, GetPredictor) {
  std::string conf_file = "modules/prediction/testdata/prediction_conf.pb.txt";
  bool ret_load_conf = ::apollo::common::util::GetProtoFromFile(conf_file,
                                                                &conf_);
  EXPECT_TRUE(ret_load_conf);
  EXPECT_TRUE(conf_.IsInitialized());

  manager_->Init(conf_);

  const ObstacleConf::PredictorType type = ObstacleConf::FREE_MOVE_PREDICTOR;
  EXPECT_TRUE(manager_->GetPredictor(type) != nullptr);
}

}  // namespace prediction
}  // namespace apollo
