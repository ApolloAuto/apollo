/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

#include "gflags/gflags.h"

#include "modules/planning/planning_base/learning_based/model_inference/proto/learning_model_inference.pb.h"
#include "modules/planning/planning_base/proto/learning_data.pb.h"
#include "modules/planning/planning_base/proto/planning_semantic_map_config.pb.h"

#include "cyber/common/file.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"
#include "modules/planning/planning_base/learning_based/img_feature_renderer/birdview_img_feature_renderer.h"
#include "modules/planning/planning_base/learning_based/model_inference/trajectory_imitation_libtorch_inference.h"

namespace apollo {
namespace planning {

DECLARE_string(test_model_inference_task_config_file);
DECLARE_string(test_data_frame_file);
DEFINE_string(test_model_inference_task_config_file, "",
              "inference task config");
DEFINE_string(test_data_frame_file, "", "test data frame");

/**
 * @class ModelInferenceTest
 * @brief This is an unit test testing on inference function
 */
class ModelInferenceTest : public ::testing::Test {
 public:
  virtual ~ModelInferenceTest() = default;
  virtual void SetUp() {
    FLAGS_map_dir = "/apollo/modules/map/data/sunnyvale_with_two_offices";
    FLAGS_base_map_filename = "base_map.bin";
  }
};

TEST_F(ModelInferenceTest, trajectory_imitation_libtorch_inference) {
  FLAGS_test_model_inference_task_config_file =
      "/apollo/modules/planning/planning_base/testdata/model_inference_test/"
      "test_libtorch_inference_task_config.pb.txt";
  FLAGS_test_data_frame_file =
      "/apollo/modules/planning/planning_base/testdata/model_inference_test/"
      "learning_data_sunnyvale_with_two_offices.bin";
  FLAGS_planning_birdview_img_feature_renderer_config_file =
      "/apollo/modules/planning/planning_component/conf/"
      "planning_semantic_map_config.pb.txt";

  LearningModelInferenceTaskConfig config;
  ACHECK(apollo::cyber::common::GetProtoFromFile(
      FLAGS_test_model_inference_task_config_file, &config))
      << "Failed to load config file "
      << FLAGS_test_model_inference_task_config_file;

  LearningDataFrame test_data_frame;
  ACHECK(apollo::cyber::common::GetProtoFromFile(FLAGS_test_data_frame_file,
                                                 &test_data_frame))
      << "Failed to load data frame file " << FLAGS_test_data_frame_file;

  PlanningSemanticMapConfig renderer_config;
  ACHECK(apollo::cyber::common::GetProtoFromFile(
      FLAGS_planning_birdview_img_feature_renderer_config_file,
      &renderer_config))
      << "Failed to load renderer config"
      << FLAGS_planning_birdview_img_feature_renderer_config_file;

  BirdviewImgFeatureRenderer::Instance()->Init(renderer_config);

  std::unique_ptr<ModelInference> trajectory_imitation_libtorch_inference =
      std::unique_ptr<ModelInference>(
          new TrajectoryImitationLibtorchInference(config));
  ACHECK(trajectory_imitation_libtorch_inference->LoadModel())
      << "Failed to load model in libtorch inference";
  ACHECK(trajectory_imitation_libtorch_inference->DoInference(&test_data_frame))
      << "Failed to inference trajectory_imitation_model";
}

}  // namespace planning
}  // namespace apollo

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  ::google::ParseCommandLineFlags(&argc, &argv, true);
  return RUN_ALL_TESTS();
}
