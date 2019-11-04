/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/prediction/submodules/predictor_submodule.h"

#include <utility>

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/adapters/proto/adapter_config.pb.h"
#include "modules/common/time/time.h"
#include "modules/common/util/message_util.h"
#include "modules/prediction/common/prediction_system_gflags.h"
#include "modules/prediction/predictor/predictor_manager.h"

namespace apollo {
namespace prediction {

using apollo::common::time::Clock;

PredictorSubmodule::~PredictorSubmodule() {}

std::string PredictorSubmodule::Name() const {
  return FLAGS_evaluator_submodule_name;
}

bool PredictorSubmodule::Init() {
  if (!MessageProcess::InitEvaluators()) {
    return false;
  }
  predictor_writer_ =
      node_->CreateWriter<PredictionObstacles>(FLAGS_prediction_topic);
  return true;
}

bool PredictorSubmodule::Proc(
    const std::shared_ptr<EvaluatorOutput>& evaluator_output,
    const std::shared_ptr<ADCTrajectoryContainer>& adc_trajectory_container) {
  const apollo::common::Header& perception_header =
      evaluator_output->submodule_output().perception_header();
  const apollo::common::ErrorCode& perception_error_code =
      evaluator_output->submodule_output().perception_error_code();
  const double frame_start_time =
      evaluator_output->submodule_output().frame_start_time();
  ObstaclesContainer obstacles_container(evaluator_output->submodule_output());
  PredictorManager::Instance()->Run(adc_trajectory_container.get(),
                                    &obstacles_container);
  PredictionObstacles prediction_obstacles =
      PredictorManager::Instance()->prediction_obstacles();

  prediction_obstacles.set_end_timestamp(Clock::NowInSeconds());
  prediction_obstacles.mutable_header()->set_lidar_timestamp(
      perception_header.lidar_timestamp());
  prediction_obstacles.mutable_header()->set_camera_timestamp(
      perception_header.camera_timestamp());
  prediction_obstacles.mutable_header()->set_radar_timestamp(
      perception_header.radar_timestamp());
  prediction_obstacles.set_perception_error_code(perception_error_code);
  prediction_obstacles.set_start_timestamp(frame_start_time);

  common::util::FillHeader(node_->Name(), &prediction_obstacles);
  predictor_writer_->Write(
      std::make_shared<PredictionObstacles>(prediction_obstacles));

  return true;
}

}  // namespace prediction
}  // namespace apollo
