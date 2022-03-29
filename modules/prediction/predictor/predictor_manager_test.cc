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

#include "modules/prediction/predictor/predictor_manager.h"

#include "cyber/common/file.h"
#include "modules/prediction/common/kml_map_based_test.h"
#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/container/container_manager.h"
#include "modules/prediction/container/obstacles/obstacles_container.h"
#include "modules/prediction/evaluator/evaluator_manager.h"

namespace apollo {
namespace prediction {

using apollo::common::adapter::AdapterConfig;

class PredictorManagerTest : public KMLMapBasedTest {
 public:
  void SetUp() override {
    std::string file =
        "modules/prediction/testdata/single_perception_vehicle_onlane.pb.txt";
    ACHECK(cyber::common::GetProtoFromFile(file, &perception_obstacles_));

    container_manager_.reset(new ContainerManager());
    evaluator_manager_.reset(new EvaluatorManager());
    predictor_manager_.reset(new PredictorManager());
  }

 protected:
  apollo::perception::PerceptionObstacles perception_obstacles_;
  common::adapter::AdapterManagerConfig adapter_conf_;
  PredictionConf prediction_conf_;
  std::unique_ptr<ContainerManager> container_manager_;
  std::unique_ptr<EvaluatorManager> evaluator_manager_;
  std::unique_ptr<PredictorManager> predictor_manager_;
};

TEST_F(PredictorManagerTest, General) {
  FLAGS_enable_trim_prediction_trajectory = false;
  std::string conf_file = "modules/prediction/testdata/adapter_conf.pb.txt";
  bool ret_load_conf =
      cyber::common::GetProtoFromFile(conf_file, &adapter_conf_);
  EXPECT_TRUE(ret_load_conf);
  EXPECT_TRUE(adapter_conf_.IsInitialized());

  container_manager_->Init(adapter_conf_);
  evaluator_manager_->Init(prediction_conf_);
  predictor_manager_->Init(prediction_conf_);

  auto obstacles_container =
      container_manager_->GetContainer<ObstaclesContainer>(
          AdapterConfig::PERCEPTION_OBSTACLES);
  CHECK_NOTNULL(obstacles_container);
  obstacles_container->Insert(perception_obstacles_);

  auto adc_trajectory_container =
      container_manager_->GetContainer<ADCTrajectoryContainer>(
          AdapterConfig::PLANNING_TRAJECTORY);

  evaluator_manager_->Run(adc_trajectory_container, obstacles_container);
  predictor_manager_->Run(perception_obstacles_, adc_trajectory_container,
                          obstacles_container);

  const PredictionObstacles& prediction_obstacles =
      predictor_manager_->prediction_obstacles();
  EXPECT_EQ(prediction_obstacles.prediction_obstacle_size(), 1);
}

}  // namespace prediction
}  // namespace apollo
