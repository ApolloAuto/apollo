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

#include "cyber/time/time.h"

#include "cyber/time/clock.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/adapters/proto/adapter_config.pb.h"
#include "modules/common/util/message_util.h"
#include "modules/prediction/common/prediction_system_gflags.h"

namespace apollo {
namespace prediction {

using apollo::cyber::Clock;
using apollo::perception::PerceptionObstacles;

std::string PredictorSubmodule::Name() const {
  return FLAGS_evaluator_submodule_name;
}

bool PredictorSubmodule::Init() {
  predictor_manager_.reset(new PredictorManager());

  PredictionConf prediction_conf;
  if (!ComponentBase::GetProtoConfig(&prediction_conf)) {
    AERROR << "Unable to load prediction conf file: "
           << ComponentBase::ConfigFilePath();
    return false;
  }
  ADEBUG << "Prediction config file is loaded into: "
         << prediction_conf.ShortDebugString();
  if (!MessageProcess::InitPredictors(predictor_manager_.get(),
                                      prediction_conf)) {
    return false;
  }
  predictor_writer_ = node_->CreateWriter<PredictionObstacles>(
      prediction_conf.topic_conf().prediction_topic());
  return true;
}

bool PredictorSubmodule::Proc(
    const std::shared_ptr<PerceptionObstacles>& perception_obstacles,
    const std::shared_ptr<ADCTrajectoryContainer>& adc_trajectory_container,
    const std::shared_ptr<SubmoduleOutput>& submodule_output) {
  const apollo::common::Header& perception_header =
      perception_obstacles->header();
  const apollo::common::ErrorCode& perception_error_code =
      perception_obstacles->error_code();
  const apollo::cyber::Time& frame_start_time =
      submodule_output->frame_start_time();
  ObstaclesContainer obstacles_container(*submodule_output);
  predictor_manager_->Run(*perception_obstacles, adc_trajectory_container.get(),
                          &obstacles_container);
  PredictionObstacles prediction_obstacles =
      predictor_manager_->prediction_obstacles();

  prediction_obstacles.set_end_timestamp(Clock::NowInSeconds());
  prediction_obstacles.mutable_header()->set_lidar_timestamp(
      perception_header.lidar_timestamp());
  prediction_obstacles.mutable_header()->set_camera_timestamp(
      perception_header.camera_timestamp());
  prediction_obstacles.mutable_header()->set_radar_timestamp(
      perception_header.radar_timestamp());
  prediction_obstacles.set_perception_error_code(perception_error_code);

  common::util::FillHeader(node_->Name(), &prediction_obstacles);
  predictor_writer_->Write(prediction_obstacles);

  const apollo::cyber::Time& end_time = Clock::Now();
  ADEBUG << "End to end time = "
         << (end_time - frame_start_time).ToSecond() * 1000 << " ms";

  return true;
}

}  // namespace prediction
}  // namespace apollo
